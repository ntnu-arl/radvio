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

#ifndef RADVIO_RADARTARGET_HPP_
#define RADVIO_RADARTARGET_HPP_

#define PCL_NO_PRECOMPILE

#include <vector>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "radvio/point.hpp"

namespace radvio
{
struct Target
{
  double x;
  double y;
  double z;
  double radial_speed;

  double range;
  double azimuth;
  double elevation;

  Eigen::Vector3d xyz;
  Eigen::Vector3d bearing;

  Target(const double x, const double y, const double z, const double radial_speed)
    : x(x), y(y), z(z), radial_speed(radial_speed)
  {
    range = std::sqrt(x * x + y * y + z * z);
    azimuth = std::atan2(y, x);
    elevation = std::atan2(z, std::sqrt(x * x + y * y));

    xyz = Eigen::Vector3d(x, y, z);
    bearing = xyz / xyz.norm();
  }
};

typedef std::vector<Target> TargetVector;

TargetVector fromRos(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  pcl::PointCloud<radar::zadarPoint> cloud;
  pcl::fromROSMsg(*msg, cloud);

  TargetVector targets;
  targets.reserve(cloud.points.size());
  for (const auto& p : cloud.points)
  {
    targets.emplace_back(p.x, p.y, p.z, p.doppler);
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
    p.velocity = t.radial_speed;

    cloud.points.push_back(p);
  }

  return cloud;
}

}  // namespace radvio

#endif /* RADVIO_RADARTARGET_HPP_ */