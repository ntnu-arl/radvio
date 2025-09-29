#ifndef RADARTARGET_HPP_
#define RADARTARGET_HPP_

#define PCL_NO_PRECOMPILE

#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "rovio/point.hpp"

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
    xyz = Eigen::Vector3d(x, y, z);
    bearing = xyz / xyz.norm();
  }
};

typedef std::vector<Target> TargetVector;

TargetVector fromRos(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<radar::mmWavePoint> cloud;
  pcl::fromROSMsg(*msg, cloud);

  TargetVector targets;
  targets.reserve(cloud.points.size());
  for (const auto& p : cloud.points)
  {
    targets.emplace_back(p.x, p.y, p.z, p.velocity);
  }

  return targets;
}

}  // namespace rovio

#endif