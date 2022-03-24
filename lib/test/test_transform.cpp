// Copyright 2022 Takeshi Ishita
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
//    * Neither the name of the Takeshi Ishita nor the names of its
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

#include <gmock/gmock.h>

#include "lidar_feature_library/transform.hpp"

TEST(Transform, MakeIsometry3d)
{
  const Eigen::Quaterniond q = Eigen::Quaterniond(-1., 1., 1., 1).normalized();
  const Eigen::Vector3d t(1., -1., 2.);
  const Eigen::Isometry3d transform = MakeIsometry3d(q, t);

  const Eigen::Vector3d p(2., 4., 1.);

  EXPECT_THAT((q * p + t - transform * p).norm(), testing::Le(1e-3));
}

TEST(Transform, TransformPointCloud)
{
  auto to_vector = [](const pcl::PointXYZ & p) {
    return Eigen::Vector3d(p.x, p.y, p.z);
  };

  const Eigen::Quaterniond q = Eigen::Quaterniond(-1., 1., 1., -1.).normalized();
  const Eigen::Vector3d t(1., 2., 4.);

  const Eigen::Affine3d transform = MakeIsometry3d(q, t);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->push_back(pcl::PointXYZ(1., 2., 0.));
  cloud->push_back(pcl::PointXYZ(2., 4., 3.));
  const auto transformed = TransformPointCloud<pcl::PointXYZ>(transform, cloud);

  EXPECT_THAT(
    (q * to_vector(cloud->at(0)) + t - to_vector(transformed->at(0))).norm(),
    testing::Le(1e-4));

  EXPECT_THAT(
    (q * to_vector(cloud->at(1)) + t - to_vector(transformed->at(1))).norm(),
    testing::Le(1e-4));
}
