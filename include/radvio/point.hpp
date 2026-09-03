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

#ifndef RADVIO_POINT_HPP_
#define RADVIO_POINT_HPP_

#include <pcl/point_types.h>

namespace radvio
{
  namespace radar
  {
    struct mmWavePoint
    {
      PCL_ADD_POINT4D; // position [m]
      float intensity; // SNR [dB]
      float velocity;  // Doppler speed [m/s]
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct zadarPoint
    {
      PCL_ADD_POINT4D; // position [m]
      float snr;
      float range;
      float noise;
      float doppler;
      float adjusted_doppler;
      uint32_t frame_num;
      uint8_t is_static;
      uint8_t removed;
      uint32_t subframe_index;
      uint32_t fence_id;
      float power;
      PCL_MAKE_ALIGNED_OPERATOR_NEW
    };
  } // namespace radar
} // namespace radvio

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
  radvio::radar::mmWavePoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (float, velocity, velocity)
)
// clang-format on
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(
  radvio::radar::zadarPoint,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, snr, snr)
  (float, range, range)
  (float, noise, noise)
  (float, doppler, doppler)
  (float, adjusted_doppler, adjusted_doppler)
  (uint32_t, frame_num, frame_num)
  (uint8_t, is_static, is_static)
  (uint8_t, removed, removed)
  (uint32_t, subframe_index, subframe_index)
  (uint32_t, fence_id, fence_id)
  (float, power, power)
)
// clang-format on

#endif /* RADVIO_POINT_HPP_ */
