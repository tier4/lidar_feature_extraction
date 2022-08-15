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

#ifndef LIDAR_FEATURE_LOCALIZATION__POINT_TO_VECTOR_HPP_
#define LIDAR_FEATURE_LOCALIZATION__POINT_TO_VECTOR_HPP_

#include <Eigen/Core>
#include <pcl/point_types.h>

#include "lidar_feature_library/point_type.hpp"


class PointXYZCRToVector
{
public:
  using PointType = PointXYZCR;

  static Eigen::VectorXd Convert(const PointXYZCR & p)
  {
    return Eigen::Vector4d(p.x, p.y, p.z, p.curvature);
  }

  static size_t NumDimension()
  {
    return 4;
  }
};

class PointXYZCRToXYZVector
{
public:
  using PointType = PointXYZCR;

  static Eigen::VectorXd Convert(const PointXYZCR & p)
  {
    return Eigen::Vector3d(p.x, p.y, p.z);
  }

  static size_t NumDimension()
  {
    return 3;
  }
};

class PointXYZToVector
{
public:
  using PointType = pcl::PointXYZ;

  static Eigen::VectorXd Convert(const pcl::PointXYZ & p)
  {
    return Eigen::Vector3d(p.x, p.y, p.z);
  }

  static size_t NumDimension()
  {
    return 3;
  }
};

#endif  // LIDAR_FEATURE_LOCALIZATION__POINT_TO_VECTOR_HPP_
