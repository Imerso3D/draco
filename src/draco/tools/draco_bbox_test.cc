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

#include "draco/tools/draco_bbox.h"

#include "draco/core/draco_test_base.h"
#include "draco/core/draco_test_utils.h"
namespace draco {
TEST(BoundingBoxTest, CropMeshTest) {
  auto sphere = ReadMeshFromTestFile("test_sphere.obj", true);
  ASSERT_NE(sphere, nullptr) << "Failed to load test model test_sphere.obj";
  BoundingBox bbox;
  bbox.min = std::array<float, 3>{-10.0, 0.0, 0.0};
  bbox.max = std::array<float, 3>{10.0,  0.5, 0.5};
  auto orig_faces = sphere->num_faces();
  auto orig_points = sphere->num_points();

  CropMesh(bbox, *sphere);

  EXPECT_LT(sphere->num_faces(), orig_faces);
  EXPECT_LT(sphere->num_points(), orig_points);
}

TEST(BoundingBoxTest, NoCropMesh) {
  auto sphere = ReadMeshFromTestFile("test_sphere.obj", true);
  ASSERT_NE(sphere, nullptr) << "Failed to load test model test_sphere.obj";
  BoundingBox bbox;
  bbox.min = {-10.0, -10.0, -10.0};
  bbox.max = {-9.0,  -9.0, -9.0};

  CropMesh(bbox, *sphere);

  EXPECT_EQ(0, sphere->num_faces());
  EXPECT_EQ(0, sphere->num_points());
}

TEST(BoundingBoxTest, TotalCrop) {
  auto sphere = ReadMeshFromTestFile("test_sphere.obj", true);
  ASSERT_NE(sphere, nullptr) << "Failed to load test model test_sphere.obj";
  BoundingBox bbox;
  bbox.min = {-10.0, -10.0, -10.0};
  bbox.max = {10.0,  10.0, 10.0};

  auto orig_faces = sphere->num_faces();
  auto orig_points = sphere->num_points();
  CropMesh(bbox, *sphere);

  EXPECT_EQ(sphere->num_faces(), orig_faces);
  EXPECT_EQ(sphere->num_points(), orig_points);
}

TEST(BoundingBoxTest, CropCloudTest) {
  auto sphere = ReadPointCloudFromTestFile("test_sphere.obj");
  ASSERT_NE(sphere, nullptr) << "Failed to load test model test_sphere.obj";
  BoundingBox bbox;
  bbox.min = std::array<float, 3>{-10.0, 0.0, 0.0};
  bbox.max = std::array<float, 3>{10.0,  0.5, 0.5};
  auto orig_points = sphere->num_points();

  CropCloud(bbox, *sphere);

  EXPECT_LT(sphere->num_points(), orig_points);
}

TEST(BoundingBoxTest, NoCropCloud) {
  auto sphere = ReadPointCloudFromTestFile("test_sphere.obj");
  ASSERT_NE(sphere, nullptr) << "Failed to load test model test_sphere.obj";
  BoundingBox bbox;
  bbox.min = {-10.0, -10.0, -10.0};
  bbox.max = {-9.0,  -9.0, -9.0};

  CropCloud(bbox, *sphere);

  EXPECT_EQ(0, sphere->num_points());
}

TEST(BoundingBoxTest, TotalCropCloud) {
  auto sphere = ReadPointCloudFromTestFile("test_sphere.obj");
  ASSERT_NE(sphere, nullptr) << "Failed to load test model test_sphere.obj";
  BoundingBox bbox;
  bbox.min = {-10.0, -10.0, -10.0};
  bbox.max = {10.0,  10.0, 10.0};

  auto orig_points = sphere->num_points();
  CropCloud(bbox, *sphere);

  EXPECT_EQ(sphere->num_points(), orig_points);
}
}