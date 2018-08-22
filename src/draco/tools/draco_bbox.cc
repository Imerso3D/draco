// Copyright 2016 The Draco Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// This tool is implemented by Anders G. Bjørnstad / Imerso AS, heavily
// based on draco_decoder.cc
#include <cinttypes>
#include <fstream>
#include <set>
#include <algorithm>

#include "draco/compression/encode.h"
#include "draco/core/cycle_timer.h"
#include "draco/io/point_cloud_io.h"
#include "draco/io/obj_encoder.h"
#include "draco/io/parser_utils.h"
#include "draco/io/ply_encoder.h"
#include "draco/tools/draco_encoder.h"
#include "draco/tools/draco_bbox.h"

namespace {

  struct Options {
    Options();

    int compression_level;
    std::string input;
    std::string output;
    std::string bbox_file;

  };

  Options::Options()
  : compression_level(7) {}

  void Usage() {
    printf("Usage: draco_bbox [options] -b bbox_file -i input\n");
    printf("\n");
    printf("Main options:\n");
    printf("  -h | -?               show help.\n");
    printf("  -o <output>           output file name.\n");
    printf("\n");
    printf("bbox_file should be a PLY or OBJ point cloud with at least two points\n");
  }

  int ReturnError(const draco::Status &status) {
    printf("Failed to decode the input file %s\n", status.error_msg());
    return -1;
  }



}  // namespace

int main(int argc, char **argv) {
  Options options;
  const int argc_check = argc - 1;

  for (int i = 1; i < argc; ++i) {
    if (!strcmp("-h", argv[i]) || !strcmp("-?", argv[i])) {
      Usage();
      return 0;
    } else if (!strcmp("-i", argv[i]) && i < argc_check) {
      options.input = argv[++i];
    } else if (!strcmp("-o", argv[i]) && i < argc_check) {
      options.output = argv[++i];
    } else if (!strcmp("-b", argv[i]) && i < argc_check) {
      options.bbox_file = argv[++i];
    }
  }
  if (argc < 3 || options.input.empty()) {
    Usage();
    return -1;
  }

  std::ifstream input_file(options.input, std::ios::binary);
  if (!input_file) {
    printf("Failed opening the input file.\n");
    return -1;
  }

  // Read the file stream into a buffer.
  std::streampos file_size = 0;
  input_file.seekg(0, std::ios::end);
  file_size = input_file.tellg() - file_size;
  input_file.seekg(0, std::ios::beg);
  std::vector<char> data(file_size);
  input_file.read(data.data(), file_size);

  if (data.empty()) {
    printf("Empty input file.\n");
    return -1;
  }

  // Read bounding box point cloud file
  draco::BoundingBox bbox;
  {
    auto maybe_pc = draco::ReadPointCloudFromFile(options.bbox_file);
    if (!maybe_pc.ok()) {
      printf("Failed loading the bbox point cloud: %s.\n",
             maybe_pc.status().error_msg());
      return -1;
    }
    const draco::PointCloud* pc = maybe_pc.value().get();
    const draco::PointAttribute *const att =
      pc->GetNamedAttribute(draco::GeometryAttribute::POSITION);

    std::vector<std::array<float, 3>> points(pc->num_points());
    for (draco::PointIndex i(0); i < pc->num_points(); i++) {
      std::array<float, 3> pos{-1, -1, -1};
      if (!att->ConvertValue<float>(att->mapped_index(i), 3, &pos[0])) {
        printf("Failed to get XYZ position from a point\n");
        return -1;
      }
      points[i.value()] = pos;
    }
    auto x_comp = [](const std::array<float, 3>& a, const std::array<float, 3>& b){ return a[0] < b[0]; };
    auto y_comp = [](const std::array<float, 3>& a, const std::array<float, 3>& b){ return a[1] < b[1]; };
    auto z_comp = [](const std::array<float, 3>& a, const std::array<float, 3>& b){ return a[2] < b[2]; };
    bbox.min = {(*std::min_element(points.begin(), points.end(), x_comp))[0],
                (*std::min_element(points.begin(), points.end(), y_comp))[1],
                (*std::min_element(points.begin(), points.end(), z_comp))[2]};
    bbox.max = {(*std::max_element(points.begin(), points.end(), x_comp))[0],
                (*std::max_element(points.begin(), points.end(), y_comp))[1],
                (*std::max_element(points.begin(), points.end(), z_comp))[2]};

  }

  // Create a draco decoding buffer. Note that no data is copied in this step.
  draco::DecoderBuffer buffer;
  buffer.Init(data.data(), data.size());

  draco::CycleTimer timer;
  // Decode the input data into a geometry.
  std::unique_ptr<draco::PointCloud> pc;
  draco::Mesh *mesh = nullptr;
  auto type_statusor = draco::Decoder::GetEncodedGeometryType(&buffer);
  if (!type_statusor.ok()) {
    return ReturnError(type_statusor.status());
  }
  const draco::EncodedGeometryType geom_type = type_statusor.value();
  if (geom_type == draco::TRIANGULAR_MESH) {
    timer.Start();
    draco::Decoder decoder;
    auto statusor = decoder.DecodeMeshFromBuffer(&buffer);
    if (!statusor.ok()) {
      return ReturnError(statusor.status());
    }
    std::unique_ptr<draco::Mesh> in_mesh = std::move(statusor).value();
    timer.Stop();
    if (in_mesh) {
      mesh = in_mesh.get();
      pc = std::move(in_mesh);
    }
  } else if (geom_type == draco::POINT_CLOUD) {
    // Failed to decode it as mesh, so let's try to decode it as a point cloud.
    timer.Start();
    draco::Decoder decoder;
    auto statusor = decoder.DecodePointCloudFromBuffer(&buffer);
    if (!statusor.ok()) {
      return ReturnError(statusor.status());
    }
    pc = std::move(statusor).value();
    timer.Stop();
  }

  if (pc == nullptr) {
    printf("Failed to decode the input file.\n");
    return -1;
  }//

  if (options.output.empty()) {
    // Create a default output file with a _cropped suffix
    long start = std::max<long>(0, options.input.length() - 4);
    if (options.input.compare(start, 4, ".drc") == 0 || options.input.compare(start, 4, ".DRC") == 0) {
      options.output = options.input.substr(0, start) + "_cropped.drc";
    } else {
      // Our simple file extension stripper didn't work, just append to input instead
      options.output = options.input + "cropped.drc";
    }
  }

  draco::Encoder encoder;
  // Convert compression level to speed (that 0 = slowest, 10 = fastest).
  const int speed = 10 - options.compression_level;
  encoder.SetSpeedOptions(speed, speed);


  if (mesh) {
    printf("Original mesh has %d faces, %d points...\n", mesh->num_faces(), mesh->num_points());
    auto status = draco::CropMesh(bbox, *mesh);
    if (status.code() != draco::Status::Code::OK) {
      printf("Failed to crop mesh: %s", status.error_msg());
      return -1;
    }
    printf("Filtered mesh has %d faces, %d points\n", mesh->num_faces(), mesh->num_points());

    if (draco::EncodeMeshToFile(*mesh, options.output, &encoder) != 0) {
      printf("Failed to encode mesh to file\n");
      return -1;
    }
  } else {
    printf("Original cloud has %d points...\n", pc->num_points());
    auto status = draco::CropCloud(bbox, *pc);
    if (status.code() != draco::Status::Code::OK) {
      printf("Failed to crop cloud: %s", status.error_msg());
      return -1;
    }
    printf("Filtered cloud has %d points...\n", pc->num_points());

    if (draco::EncodePointCloudToFile(*pc, options.output, &encoder) != 0) {
      printf("Failed to encode point cloud to file\n");
      return -1;
    }
  }

  printf("Wrote filtered mesh/cloud to %s\n", options.output.c_str());
  return 0;
}

