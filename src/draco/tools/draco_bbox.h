
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

#ifndef DRACO_DRACO_BBOX_H
#define DRACO_DRACO_BBOX_H

#include <array>
#include "draco/core/status.h"
#include "draco/mesh/mesh.h"

namespace draco {
  struct Triangle3D {
    Triangle3D();

    std::array<float, 3> v0;
    std::array<float, 3> v1;
    std::array<float, 3> v2;
  };

  Triangle3D::Triangle3D() {}

  struct BoundingBox {
    BoundingBox();

    std::array<float, 3> min;
    std::array<float, 3> max;
  };

  BoundingBox::BoundingBox() {}

  bool pointInside(std::array<float, 3> point, const BoundingBox& bbox) {
    return (point[0] >= bbox.min[0] && point[0] <= bbox.max[0]
            && point[1] >= bbox.min[1] && point[1] <= bbox.max[1]
            && point[2] >= bbox.min[2] && point[2] <= bbox.max[2]);
  }

  bool faceInside(const Triangle3D& face, const BoundingBox& bbox) {
    // TODO: A large triangle may have all three points outside bbox but
    //       still be partly inside. Don't bother supporting those for now.
    return pointInside(face.v0, bbox) || pointInside(face.v1, bbox) || pointInside(face.v2, bbox);
  }

  Status CropMesh(const BoundingBox& bbox, Mesh& mesh) {
    draco::IndexTypeVector<draco::FaceIndex, draco::FaceIndex> faces_to_include;
    for (draco::FaceIndex f(0); f < mesh.num_faces(); f++) {
      const draco::Mesh::Face& face = mesh.face(f);

      const draco::PointAttribute *const att =
        mesh.GetNamedAttribute(draco::GeometryAttribute::POSITION);
      if (att == nullptr || att->size() == 0)
        return Status(Status::Code::ERROR, "POSITION attribute must be valid");

      assert(face[0] < att->size());
      assert(face[1] < att->size());
      assert(face[2] < att->size());
      draco::Triangle3D tri;
      if (!att->ConvertValue<float, 3>(att->mapped_index(face[0]), &tri.v0[0]))
        return Status(Status::Code::ERROR, "Unable to convert POSITION attribute to xyz floats");
      if (!att->ConvertValue<float, 3>(att->mapped_index(face[1]), &tri.v1[0]))
        return Status(Status::Code::ERROR, "Unable to convert POSITION attribute to xyz floats");
      if (!att->ConvertValue<float, 3>(att->mapped_index(face[2]), &tri.v2[0]))
        return Status(Status::Code::ERROR, "Unable to convert POSITION attribute to xyz floats");

      if (faceInside(tri, bbox)) {
        faces_to_include.push_back(f);
      }
    }

    mesh.FilterMesh(faces_to_include);
    return Status(Status::Code::OK);
  }

  Status CropCloud(const BoundingBox& bbox, PointCloud& pc) {
    draco::IndexTypeVector<draco::PointIndex, draco::PointIndex> points_to_include;
    for (draco::PointIndex i(0); i < pc.num_points(); i++) {
      const draco::PointAttribute *const att =
        pc.GetNamedAttribute(draco::GeometryAttribute::POSITION);
      if (att == nullptr || att->size() == 0)
        return Status(Status::Code::ERROR, "POSITION attribute must be valid");

      std::array<float, 3> v;
      if (!att->ConvertValue<float, 3>(att->mapped_index(i), &v[0]))
        return Status(Status::Code::ERROR, "Unable to convert POSITION attribute to xyz floats");
      if (pointInside(v, bbox)) {
        points_to_include.push_back(i);
      }
    }

    pc.FilterCloud(points_to_include);
    return Status(Status::Code::OK);
  }
}
#endif //DRACO_DRACO_BBOX_H
