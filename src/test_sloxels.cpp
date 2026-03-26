/*
* Copyright (c) 2026, Autonomous Robots Lab, Norwegian University of Science and Technology
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <assert.h>
#include <vector>
#include <Eigen/Core>

#include "radvio/sloxels.hpp"

using namespace radvio;

constexpr double leaf_size = 0.1;
constexpr double max_sensor_range = 20;

// Test basic insertion
TEST(VoxelMapTest, BasicInsertion)
{
  std::vector<Eigen::Vector3d> points;
  points.emplace_back(1, 1, 1);
  points.emplace_back(2, 2, 2);
  points.emplace_back(11, 0, 0);

  const Eigen::Vector3d position{ 0, 0, 0 };

  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);
  map.insert(points, position);

  // Should have 3 voxels (all points are far enough apart)
  EXPECT_EQ(map.voxels().size(), 3);

  // Verify each voxel has exactly 1 point
  for (const auto& pair : map.voxels())
  {
    EXPECT_EQ(pair.second.size(), 1);
  }
}

// Test that points in the same voxel are grouped together
TEST(VoxelMapTest, SameVoxelGrouping)
{
  std::vector<Eigen::Vector3d> points;
  // These points should all fall into the same voxel (0,0,0)
  points.emplace_back(0.01, 0.01, 0.01);
  points.emplace_back(0.02, 0.02, 0.02);
  points.emplace_back(0.03, 0.03, 0.03);

  const Eigen::Vector3d position{ 0, 0, 0 };
  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);
  map.settings().min_sq_dist_in_cell = 0.0;
  map.insert(points, position);

  // Should have only 1 voxel
  EXPECT_EQ(map.voxels().size(), 1);

  // That voxel should contain 3 points
  const auto& voxel = map.voxels().begin()->second;
  EXPECT_EQ(voxel.size(), 3);
}

// Test minimum distance filtering within a cell
TEST(VoxelMapTest, MinDistanceFiltering)
{
  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);
  map.settings().set_min_dist_in_cell(0.05);  // 5cm minimum distance

  std::vector<Eigen::Vector3d> points;
  // These points are in the same voxel but very close together
  points.emplace_back(0.01, 0.01, 0.01);
  points.emplace_back(0.011, 0.011, 0.011);  // Only 1.7mm away - should be filtered
  points.emplace_back(0.08, 0.08, 0.08);     // 12cm away - should be kept

  const Eigen::Vector3d position{ 0, 0, 0 };
  map.insert(points, position);

  // Should have 1 voxel with 2 points (middle one filtered out)
  EXPECT_EQ(map.voxels().size(), 1);
  const auto& voxel = map.voxels().begin()->second;
  EXPECT_EQ(voxel.size(), 2);
}

// Test maximum points per cell
TEST(VoxelMapTest, MaxPointsPerCell)
{
  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);
  map.settings().set_max_num_points_in_cell(3);
  map.settings().set_min_dist_in_cell(0.001);  // Very small to not filter by distance

  std::vector<Eigen::Vector3d> points;
  // Add 5 points in the same voxel
  for (int i = 0; i < 5; ++i)
  {
    points.emplace_back(0.01 + i * 0.01, 0.01, 0.01);
  }

  const Eigen::Vector3d position{ 0, 0, 0 };
  map.insert(points, position);

  // Should have 1 voxel with maximum 3 points
  EXPECT_EQ(map.voxels().size(), 1);
  const auto& voxel = map.voxels().begin()->second;
  EXPECT_LE(voxel.size(), 3);
}

// Test world to voxel coordinate conversion
TEST(VoxelMapTest, WorldToVoxelConversion)
{
  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);

  // Test various points
  EXPECT_EQ(map.world_to_voxel(Eigen::Vector3d(0, 0, 0)), Eigen::Vector3i(0, 0, 0));
  EXPECT_EQ(map.world_to_voxel(Eigen::Vector3d(0.15, 0.15, 0.15)), Eigen::Vector3i(1, 1, 1));
  EXPECT_EQ(map.world_to_voxel(Eigen::Vector3d(1.0, 2.0, 3.0)), Eigen::Vector3i(10, 20, 30));
  EXPECT_EQ(map.world_to_voxel(Eigen::Vector3d(-0.05, -0.05, -0.05)), Eigen::Vector3i(-1, -1, -1));
}

// Test voxel to world coordinate conversion
TEST(VoxelMapTest, VoxelToWorldConversion)
{
  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);

  // Voxel center should be at voxel_coord * leaf_size + 0.5 * leaf_size
  Eigen::Vector3d center = map.voxel_to_world(Eigen::Vector3i(0, 0, 0));
  EXPECT_NEAR(center.x(), 0.05, 1e-9);
  EXPECT_NEAR(center.y(), 0.05, 1e-9);
  EXPECT_NEAR(center.z(), 0.05, 1e-9);

  center = map.voxel_to_world(Eigen::Vector3i(10, 20, 30));
  EXPECT_NEAR(center.x(), 1.05, 1e-9);
  EXPECT_NEAR(center.y(), 2.05, 1e-9);
  EXPECT_NEAR(center.z(), 3.05, 1e-9);
}

// Test round-trip conversion
TEST(VoxelMapTest, RoundTripConversion)
{
  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);

  Eigen::Vector3d original(1.23, 4.56, 7.89);
  Eigen::Vector3i voxel = map.world_to_voxel(original);
  Eigen::Vector3d center = map.voxel_to_world(voxel);

  // The center should be within half a voxel of the original point
  EXPECT_LT((center - original).norm(), 0.1 * sqrt(3) / 2);
}

// Test clear functionality
TEST(VoxelMapTest, Clear)
{
  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);

  std::vector<Eigen::Vector3d> points;
  points.emplace_back(1, 1, 1);
  points.emplace_back(2, 2, 2);

  map.insert(points, Eigen::Vector3d(0, 0, 0));
  EXPECT_GT(map.voxels().size(), 0);

  map.clear();
  EXPECT_EQ(map.voxels().size(), 0);
}

// Test voxel mean calculation
TEST(VoxelMapTest, VoxelMean)
{
  sloxels::SlidingVoxelMap map(1.0, 20);  // Large voxel to group points
  map.settings().set_min_dist_in_cell(0.01);

  std::vector<Eigen::Vector3d> points;
  points.emplace_back(0.1, 0.1, 0.1);
  points.emplace_back(0.2, 0.2, 0.2);
  points.emplace_back(0.3, 0.3, 0.3);

  map.insert(points, Eigen::Vector3d(0, 0, 0));

  // All points should be in voxel (0,0,0)
  const auto& voxel = map.voxels().begin()->second;
  Eigen::Vector3d mean = voxel.mean();

  // Mean should be (0.2, 0.2, 0.2)
  EXPECT_NEAR(mean.x(), 0.2, 1e-9);
  EXPECT_NEAR(mean.y(), 0.2, 1e-9);
  EXPECT_NEAR(mean.z(), 0.2, 1e-9);
}

// Test empty voxel mean
TEST(VoxelMapTest, EmptyVoxelMean)
{
  sloxels::FlatVoxel voxel;
  Eigen::Vector3d mean = voxel.mean();

  // Empty voxel should return zero
  EXPECT_EQ(mean, Eigen::Vector3d::Zero());
}

// Test large point cloud
TEST(VoxelMapTest, LargePointCloud)
{
  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);

  std::vector<Eigen::Vector3d> points;
  // Create a 10x10x10 grid of points
  for (int x = 0; x < 10; ++x)
  {
    for (int y = 0; y < 10; ++y)
    {
      for (int z = 0; z < 10; ++z)
      {
        points.emplace_back(x * 0.2, y * 0.2, z * 0.2);
      }
    }
  }

  map.insert(points, Eigen::Vector3d(0, 0, 0));

  // Should have 1000 voxels (each point in its own voxel due to 0.2m spacing)
  EXPECT_EQ(map.voxels().size(), 1000);
}

// Test multiple insertions
TEST(VoxelMapTest, MultipleInsertions)
{
  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);

  std::vector<Eigen::Vector3d> points1;
  points1.emplace_back(1, 1, 1);

  std::vector<Eigen::Vector3d> points2;
  points2.emplace_back(2, 2, 2);

  map.insert(points1, Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(map.voxels().size(), 1);

  map.insert(points2, Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(map.voxels().size(), 2);
}

// Test incremental insertion into same voxel
TEST(VoxelMapTest, IncrementalSameVoxel)
{
  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);
  map.settings().set_min_dist_in_cell(0.02);

  std::vector<Eigen::Vector3d> points1;
  points1.emplace_back(0.01, 0.01, 0.01);
  map.insert(points1, Eigen::Vector3d(0, 0, 0));

  std::vector<Eigen::Vector3d> points2;
  points2.emplace_back(0.05, 0.05, 0.05);  // Same voxel, far enough to not be filtered
  map.insert(points2, Eigen::Vector3d(0, 0, 0));

  EXPECT_EQ(map.voxels().size(), 1);
  const auto& voxel = map.voxels().begin()->second;
  EXPECT_EQ(voxel.size(), 2);
}

// Test negative coordinates
TEST(VoxelMapTest, NegativeCoordinates)
{
  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);

  std::vector<Eigen::Vector3d> points;
  points.emplace_back(-1, -1, -1);
  points.emplace_back(-2, -2, -2);
  points.emplace_back(1, 1, 1);

  map.insert(points, Eigen::Vector3d(0, 0, 0));

  EXPECT_EQ(map.voxels().size(), 3);
}

// Test different leaf sizes
TEST(VoxelMapTest, DifferentLeafSizes)
{
  // Small leaf size - more voxels
  sloxels::SlidingVoxelMap map_small(leaf_size, max_sensor_range);

  // Large leaf size - fewer voxels
  sloxels::SlidingVoxelMap map_large(leaf_size * 100, max_sensor_range);

  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 10; ++i)
  {
    points.emplace_back(i * 0.1, 0, 0);
  }

  map_small.insert(points, Eigen::Vector3d(0, 0, 0));
  map_large.insert(points, Eigen::Vector3d(0, 0, 0));

  // Small leaf size should have more voxels
  EXPECT_GT(map_small.voxels().size(), map_large.voxels().size());
}

// Test pruning when position changes
TEST(VoxelMapTest, Pruning)
{
  sloxels::SlidingVoxelMap map(leaf_size, 2.0);

  std::vector<Eigen::Vector3d> points;
  points.emplace_back(1, 1, 1);
  points.emplace_back(-1, -1, -1);

  map.insert(points, Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(map.voxels().size(), 2);

  points.clear();
  map.insert(points, Eigen::Vector3d(2, 2, 2));
  EXPECT_EQ(map.voxels().size(), 1);
}

// Test that the getVoxelVertices function returns the correct 8 corners
TEST(VoxelMapTest, GetVoxelVertices)
{
  sloxels::SlidingVoxelMap map(leaf_size, max_sensor_range);
  const double size = map.getLeafSize();

  // Test Voxel (1, 2, 3)
  const Eigen::Vector3i coord(1, 2, 3);

  // Calculate the minimum corner expected: (1*0.1, 2*0.1, 3*0.1) = (0.1, 0.2, 0.3)
  const Eigen::Vector3d expected_min_corner = coord.cast<double>() * size;

  // Get the vertices using the map function
  std::vector<Eigen::Vector3d> vertices;
  map.getVoxelVertices(coord, vertices);  // Assuming the optimized void function signature

  // 1. Check count
  ASSERT_EQ(vertices.size(), 8);

  // 2. Check the bounds (min and max)
  // Check the minimum corner (should be one of the vertices)
  const Eigen::Vector3d min_vertex = vertices[0]
                                         .cwiseMin(vertices[1])
                                         .cwiseMin(vertices[2])
                                         .cwiseMin(vertices[3])
                                         .cwiseMin(vertices[4])
                                         .cwiseMin(vertices[5])
                                         .cwiseMin(vertices[6])
                                         .cwiseMin(vertices[7]);

  // Check the maximum corner (should be one of the vertices)
  const Eigen::Vector3d max_vertex = vertices[0]
                                         .cwiseMax(vertices[1])
                                         .cwiseMax(vertices[2])
                                         .cwiseMax(vertices[3])
                                         .cwiseMax(vertices[4])
                                         .cwiseMax(vertices[5])
                                         .cwiseMax(vertices[6])
                                         .cwiseMax(vertices[7]);

  // The calculated minimum should match the expected minimum corner
  EXPECT_NEAR(min_vertex.x(), expected_min_corner.x(), 1e-9);
  EXPECT_NEAR(min_vertex.y(), expected_min_corner.y(), 1e-9);
  EXPECT_NEAR(min_vertex.z(), expected_min_corner.z(), 1e-9);

  // The maximum corner should be the minimum corner plus the leaf size in all axes
  EXPECT_NEAR(max_vertex.x(), expected_min_corner.x() + size, 1e-9);
  EXPECT_NEAR(max_vertex.y(), expected_min_corner.y() + size, 1e-9);
  EXPECT_NEAR(max_vertex.z(), expected_min_corner.z() + size, 1e-9);

  // 3. Test a known vertex (e.g., the center of the face facing positive Y)
  const Eigen::Vector3d expected_face_center = expected_min_corner + Eigen::Vector3d(size, 0.0, size);
  bool found_vertex = false;
  for (const auto& v : vertices)
  {
    if ((v - expected_face_center).norm() < 1e-9)
    {
      found_vertex = true;
      break;
    }
  }
  EXPECT_TRUE(found_vertex) << "Did not find expected vertex at (" << expected_face_center.transpose() << ")";
}

GTEST_API_ int main(int argc, char** argv)
{
  printf("Running main() from %s\n", __FILE__);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}