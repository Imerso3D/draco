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
#include "draco/mesh/mesh.h"

#include <array>
#include <unordered_map>
#include <set>
#include <map>

namespace draco {

using std::unordered_map;

// Shortcut for typed conditionals.
template <bool B, class T, class F>
using conditional_t = typename std::conditional<B, T, F>::type;

Mesh::Mesh() {}

void Mesh::FilterMesh(const IndexTypeVector<FaceIndex, FaceIndex>& faces_to_include) {
  // Insert into a std::set which will not contain duplicates and will be sorted
  std::set<PointIndex> unique_points;
  for (FaceIndex f_new(0); f_new < faces_to_include.size(); f_new++) {
    const FaceIndex& f_orig = faces_to_include[f_new];
    for (int32_t c = 0; c < 3; ++c) {
      unique_points.insert(faces_[f_orig][c]);
    }
  }

  // Create index mapping both ways old<=>new
  IndexTypeVector<PointIndex, PointIndex> points_to_include(unique_points.size());
  std::map<PointIndex, PointIndex> old_to_new;
  PointIndex p_new(0);
  for (PointIndex p : unique_points) {
    points_to_include[p_new] = p;
    old_to_new[p] = p_new;
    p_new++;
  }

  PointCloud::FilterCloud(points_to_include);

  // Move faces we want to keep to beginning of face list, delete the rest
  for (draco::FaceIndex f(0); f < faces_to_include.size(); f++) {
    const Face& orig_face = faces_[faces_to_include[f]];
    Face face_new_indices = {old_to_new.at(orig_face[0]),
                             old_to_new.at(orig_face[1]),
                             old_to_new.at(orig_face[2])};
    faces_[f] = face_new_indices;
  }
  faces_.resize(faces_to_include.size(), Face());
}

#ifdef DRACO_ATTRIBUTE_DEDUPLICATION_SUPPORTED
void Mesh::ApplyPointIdDeduplication(
    const IndexTypeVector<PointIndex, PointIndex> &id_map,
    const std::vector<PointIndex> &unique_point_ids) {
  PointCloud::ApplyPointIdDeduplication(id_map, unique_point_ids);
  for (FaceIndex f(0); f < num_faces(); ++f) {
    for (int32_t c = 0; c < 3; ++c) {
      faces_[f][c] = id_map[faces_[f][c]];
    }
  }
}
#endif

}  // namespace draco
