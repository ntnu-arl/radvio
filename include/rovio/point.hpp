#pragma once

#include <pcl/point_types.h>

namespace rovio
{
namespace radar
{
struct mmWavePoint
{
  PCL_ADD_POINT4D;  // position [m]
  float intensity;  // SNR [dB]
  float velocity;   // Doppler speed [m/s]
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace radar
}  // namespace rovio

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
  rovio::radar::mmWavePoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (float, velocity, velocity)
)
// clang-format on