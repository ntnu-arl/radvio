#ifndef RADARTARGET_HPP_
#define RADARTARGET_HPP_

#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace rovio
{
struct Target
{
  float x;
  float y;
  float z;
  float radial_speed;

  Eigen::Vector3d xyz;
  Eigen::Vector3d bearing;

  Target(const float x, const float y, const float z, const float radial_speed)
    : x(x), y(y), z(z), radial_speed(radial_speed)
  {
    xyz = Eigen::Vector3d(x,y,z);
    bearing = xyz / xyz.norm();
  }
};

typedef std::vector<Target> TargetVector;

TargetVector fromRos(const sensor_msgs::PointCloud2::Ptr& cloud)
{
  sensor_msgs::PointCloud2Iterator<float> it_x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(*cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> it_velocity(*cloud, "velocity");

  TargetVector targets;
  targets.reserve(cloud->width);
  for (size_t i = 0; i < cloud->width; ++i)
  {
    targets.emplace_back(*it_x, *it_y, *it_z, *it_velocity);

    ++it_x;
    ++it_y;
    ++it_z;
    ++it_velocity;
  }

  return targets;
}
}  // namespace rovio

#endif