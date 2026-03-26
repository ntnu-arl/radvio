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

#ifndef RADVIO_SLOXELS_HPP_
#define RADVIO_SLOXELS_HPP_

#include <Eigen/Core>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cmath>

#include <Eigen/StdVector>

namespace radvio
{
namespace sloxels
{
// Convenience typedefs for Eigen-aligned STL containers
template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename Key, typename Value, typename Hash = std::hash<Key>, typename Eq = std::equal_to<Key>>
using AlignedUnorderedMap =
    std::unordered_map<Key, Value, Hash, Eq, Eigen::aligned_allocator<std::pair<const Key, Value>>>;

typedef Eigen::Vector3i Key;

/// @brief Fast floor (https://stackoverflow.com/questions/824118/why-is-floor-so-slow).
/// @param pt Double vector
/// @return Floored int vector
inline Eigen::Array4i fast_floor(const Eigen::Array4d& pt)
{
  const Eigen::Array4i ncoord = pt.cast<int>();
  return ncoord - (pt < ncoord.cast<double>()).cast<int>();
}

/**
 * @brief Spatial hashing function
 * Teschner et al., "Optimized Spatial Hashing for Collision Detection of Deformable Objects", VMV2003
 */
class XORVector3iHash
{
public:
  size_t operator()(const Key& x) const
  {
    const size_t p1 = 9132043225175502913;
    const size_t p2 = 7277549399757405689;
    const size_t p3 = 6673468629021231217;
    return static_cast<size_t>((x[0] * p1) ^ (x[1] * p2) ^ (x[2] * p3));
  }
};

struct FlatVoxel
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  struct Setting
  {
    void set_min_dist_in_cell(double dist)
    {
      this->min_sq_dist_in_cell = dist * dist;
    }
    void set_max_num_points_in_cell(size_t num_points)
    {
      this->max_num_points_in_cell = num_points;
    }

    double min_sq_dist_in_cell = 0.02 * 0.02;  ///< Minimum squared distance between points in a cell.
    size_t max_num_points_in_cell = 20;        ///< Maximum number of points in a cell.
  };

  FlatVoxel()
  {
    points_.reserve(20);
  }
  size_t size() const
  {
    return points_.size();
  }

  void add(const Setting& setting, const Eigen::Vector4d& point)
  {
    if (this->points_.size() >= setting.max_num_points_in_cell ||
        std::any_of(this->points_.begin(), this->points_.end(), [&point, &setting](const auto& pt) {
          return (pt - point).squaredNorm() < setting.min_sq_dist_in_cell;
        }))
    {
      return;
    }

    this->points_.push_back(point);
  }

  /// @brief Calculate the mean point of all points in the voxel
  /// @return Mean point as Eigen::Vector3d
  Eigen::Vector3d mean() const
  {
    if (points_.empty())
    {
      return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& pt : points_)
    {
      sum += pt.head<3>();
    }
    return sum / static_cast<double>(points_.size());
  }

  AlignedVector<Eigen::Vector4d> points_;  ///< Points
};

/**
 * @brief Sliding window voxel map for robot navigation
 *
 * This class maintains a voxelized representation of the environment around a robot.
 * Voxels outside the sensor range are automatically removed during insertion.
 */
class SlidingVoxelMap
{
public:
  /**
   * @brief Constructor
   * @param leaf_size Voxel size in meters (default: 0.1m = 10cm)
   * @param max_sensor_range Maximum sensor range in meters (default: 20m)
   */
  SlidingVoxelMap(double leaf_size = 0.1, double max_sensor_range = 20.0)
    : leaf_size_(leaf_size)
    , max_sensor_range_(max_sensor_range)
    , max_sensor_range_sq_(max_sensor_range * max_sensor_range)
    , inv_leaf_size_(1.0 / leaf_size)
  {
  }

  double getLeafSize() const
  {
    return leaf_size_;
  }

  double getMaxRange() const
  {
    return max_sensor_range_;
  }

  /**
   * @brief Insert a point cloud into the voxel map
   * @param points Point cloud in world frame (each point is Eigen::Vector4d with homogeneous coordinates)
   * @param robot_position Robot position in world frame (x, y, z)
   *
   * This function inserts points into voxels and removes voxels whose mean point
   * is more than max_sensor_range away from the robot position.
   */
  void insert(const AlignedVector<Eigen::Vector4d>& points, const Eigen::Vector3d& robot_position)
  {
    XORVector3iHash hash;
    // Insert new points into voxels
    for (size_t i = 0; i < points.size(); ++i)
    {
      const Key voxel_coord = world_to_voxel(points[i].head<3>());
      voxel_map_[voxel_coord].add(voxel_setting_, points[i]);
    }

    // Remove voxels outside sensor range
    prune_distant_voxels(robot_position);
  }

  /**
   * @brief Insert a point cloud into the voxel map (alternative signature)
   * @param points Point cloud in world frame (each point is Eigen::Vector3d)
   * @param robot_position Robot position in world frame (x, y, z)
   */
  void insert(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& robot_position)
  {
    AlignedVector<Eigen::Vector4d> points_homogeneous;
    points_homogeneous.reserve(points.size());
    for (const auto& pt : points)
    {
      points_homogeneous.emplace_back(pt.x(), pt.y(), pt.z(), 1.0);
    }
    insert(points_homogeneous, robot_position);
  }
  // TODO(morten): add field for if voxel has non-empty neighbors

  /**
   * @brief Erase voxel by key
   * @param coord Key for voxel to be erased
   */
  void erase(const Key& coord)
  {
    voxel_map_.erase(coord);
  }

  /**
   * @brief Get the underlying voxel map for iteration
   * @return Const reference to the hash map of voxels
   */
  const AlignedUnorderedMap<Key, FlatVoxel, XORVector3iHash>& voxels() const
  {
    return voxel_map_;
  }

  /**
   * @brief Clear all voxels from the map
   */
  void clear()
  {
    voxel_map_.clear();
  }

  /**
   * @brief Get voxel settings
   * @return Reference to voxel settings
   */
  FlatVoxel::Setting& settings()
  {
    return voxel_setting_;
  }

  /**
   * @brief Convert world coordinates to voxel coordinates
   * @param world_point Point in world frame
   * @return Voxel coordinate
   */
  Key world_to_voxel(const Eigen::Vector3d& world_point) const
  {
    Eigen::Array4d pt_array;
    pt_array << world_point.x(), world_point.y(), world_point.z(), 0.0;
    pt_array *= inv_leaf_size_;
    const Eigen::Array4i voxel_array = fast_floor(pt_array);
    return voxel_array.head<3>();
  }

  /**
   * @brief Convert voxel coordinates to world coordinates (voxel center)
   * @param voxel_coord Voxel coordinate
   * @return Point in world frame (voxel center)
   */
  Eigen::Vector3d voxel_to_world(const Key& voxel_coord) const
  {
    return (voxel_coord.cast<double>() + Eigen::Vector3d::Constant(0.5)) * leaf_size_;
  }

  void getVoxelVertices(const Key& coord, std::vector<Eigen::Vector3d>& vertices) const
  {
    constexpr int num_vertices = 8;

    // The minimum corner is (coord * leaf_size)
    const Eigen::Vector3d min_corner = coord.cast<double>() * leaf_size_;

    // dx, dy, dz
    static const Eigen::Vector3d offsets[num_vertices] = {
      Eigen::Vector3d(0.0, 0.0, 0.0),                      // 0: The minimum corner itself
      Eigen::Vector3d(leaf_size_, 0.0, 0.0),               // 1
      Eigen::Vector3d(0.0, leaf_size_, 0.0),               // 2
      Eigen::Vector3d(0.0, 0.0, leaf_size_),               // 3
      Eigen::Vector3d(leaf_size_, leaf_size_, 0.0),        // 4
      Eigen::Vector3d(leaf_size_, 0.0, leaf_size_),        // 5
      Eigen::Vector3d(0.0, leaf_size_, leaf_size_),        // 6
      Eigen::Vector3d(leaf_size_, leaf_size_, leaf_size_)  // 7: The maximum corner
    };

    vertices.clear();
    vertices.reserve(num_vertices);
    for (int i = 0; i < num_vertices; ++i)
    {
      vertices.emplace_back(min_corner + offsets[i]);
    }
  }

private:
  /**
   * @brief Remove voxels whose mean point is farther than max_sensor_range from robot
   * @param robot_position Robot position in world frame
   */
  void prune_distant_voxels(const Eigen::Vector3d& robot_position)
  {
    auto it = voxel_map_.begin();
    while (it != voxel_map_.end())
    {
      const Eigen::Vector3d voxel_mean = it->second.mean();
      const double dist_sq = (voxel_mean - robot_position).squaredNorm();

      if (dist_sq > max_sensor_range_sq_)
      {
        it = voxel_map_.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  const double leaf_size_;            ///< Voxel size in meters
  const double max_sensor_range_;     ///< Maximum sensor range in meters
  const double max_sensor_range_sq_;  ///< Squared maximum sensor range
  const double inv_leaf_size_;        ///< Inverse of leaf size for fast division

  FlatVoxel::Setting voxel_setting_;                                ///< Settings for voxel behavior
  AlignedUnorderedMap<Key, FlatVoxel, XORVector3iHash> voxel_map_;  ///< Hash map of voxels
};

}  // namespace sloxels
}  // namespace radvio

#endif /* RADVIO_SLOXELS_HPP_ */
