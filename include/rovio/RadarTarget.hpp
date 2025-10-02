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
  float intensity;
  float radial_speed;

  Eigen::Vector3d xyz;
  Eigen::Vector3d bearing;

  Target(const float x, const float y, const float z, const float intensity, const float radial_speed)
    : x(x), y(y), z(z), intensity(intensity), radial_speed(radial_speed)
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
    targets.emplace_back(p.x, p.y, p.z, p.intensity, p.velocity);
  }

  return targets;
}

pcl::PointCloud<radar::mmWavePoint> toPcl(const TargetVector& targets)
{
  pcl::PointCloud<radar::mmWavePoint> cloud;

  cloud.points.reserve(targets.size());
  for (const Target& t : targets)
  {
    radar::mmWavePoint p;
    p.x = t.x;
    p.y = t.y;
    p.z = t.z;
    p.intensity = t.intensity;
    p.velocity = t.radial_speed;

    cloud.points.push_back(p);
  }

  return cloud;
}

}  // namespace rovio

#endif