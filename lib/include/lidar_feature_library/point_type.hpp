// Copyright 2022 Tixiao Shan, Takeshi Ishita
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Tixiao Shan, Takeshi Ishita nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef LIDAR_FEATURE_LIBRARY__POINT_TYPE_HPP_
#define LIDAR_FEATURE_LIBRARY__POINT_TYPE_HPP_

#include <pcl/point_types.h>
#include <Eigen/Core>

struct PointXYZCR
{
  inline PointXYZCR(float _x, float _y, float _z, float _curvature, std::uint16_t _ring)
  {
    x = _x; y = _y; z = _z;
    data[3] = 1.0f;
    curvature = _curvature;
    ring = _ring;
  }

  inline PointXYZCR()
  : PointXYZCR(0.f, 0.f, 0.f, 0.f, 0)
  {
  }

  PCL_ADD_POINT4D
  float curvature;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZCR,
  (float, x, x)(float, y, y)(float, z, z)(float, curvature, curvature)(std::uint16_t, ring, ring)
)

struct PointXYZIR
{
  inline PointXYZIR(float _x, float _y, float _z, float _intensity, std::uint16_t _ring)
  {
    x = _x; y = _y; z = _z;
    data[3] = 1.0f;
    intensity = _intensity;
    ring = _ring;
  }

  inline PointXYZIR()
  : PointXYZIR(0.f, 0.f, 0.f, 0.f, 0)
  {
  }

  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIR,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)
)

#endif  // LIDAR_FEATURE_LIBRARY__POINT_TYPE_HPP_
