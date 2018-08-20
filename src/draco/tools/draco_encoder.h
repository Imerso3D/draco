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
#ifndef DRACO_DRACO_ENCODER_H
#define DRACO_DRACO_ENCODER_H
#include "draco/compression/encode.h"

namespace draco {
  int EncodePointCloudToFile(const draco::PointCloud &pc, const std::string &file,
                             draco::Encoder *encoder) {
    draco::CycleTimer timer;
    // Encode the geometry.
    draco::EncoderBuffer buffer;
    timer.Start();
    const draco::Status status = encoder->EncodePointCloudToBuffer(pc, &buffer);
    if (!status.ok()) {
      printf("Failed to encode the point cloud.\n");
      printf("%s\n", status.error_msg());
      return -1;
    }
    timer.Stop();
    // Save the encoded geometry into a file.
    std::ofstream out_file(file, std::ios::binary);
    if (!out_file) {
      printf("Failed to create the output file.\n");
      return -1;
    }
    out_file.write(buffer.data(), buffer.size());
    printf("Encoded point cloud saved to %s (%"
    PRId64
    " ms to encode).\n",
      file.c_str(), timer.GetInMs());
    printf("\nEncoded size = %zu bytes\n\n", buffer.size());
    return 0;
  }

  int EncodeMeshToFile(const draco::Mesh &mesh, const std::string &file,
                       draco::Encoder *encoder) {
    draco::CycleTimer timer;
    // Encode the geometry.
    draco::EncoderBuffer buffer;
    timer.Start();
    const draco::Status status = encoder->EncodeMeshToBuffer(mesh, &buffer);
    if (!status.ok()) {
      printf("Failed to encode the mesh.\n");
      printf("%s\n", status.error_msg());
      return -1;
    }
    timer.Stop();
    // Save the encoded geometry into a file.
    std::ofstream out_file(file, std::ios::binary);
    if (!out_file) {
      printf("Failed to create the output file.\n");
      return -1;
    }
    out_file.write(buffer.data(), buffer.size());
    printf("Encoded mesh saved to %s (%"
    PRId64
    " ms to encode).\n", file.c_str(),
      timer.GetInMs());
    printf("\nEncoded size = %zu bytes\n\n", buffer.size());
    return 0;
  }
}
#endif //DRACO_DRACO_ENCODER_H
