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


#include <gmock/gmock.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_feature_extraction/range.hpp"


template<typename T>
bool Equal(const T & a, const T & b)
{
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

TEST(Range, IsInInclusiveRange) {
  EXPECT_TRUE(IsInInclusiveRange(3., 1., 5.));
  EXPECT_TRUE(IsInInclusiveRange(1., 1., 5.));
  EXPECT_TRUE(IsInInclusiveRange(5., 1., 5.));

  EXPECT_FALSE(IsInInclusiveRange(0., 1., 5.));
  EXPECT_FALSE(IsInInclusiveRange(6., 1., 5.));
}

TEST(Range, Range) {
  auto norm = [](const pcl::PointXYZ & p) {
      return std::sqrt(p.x * p.x + p.y * p.y);
    };

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->push_back(pcl::PointXYZ(3., 4., 0.));
  cloud->push_back(pcl::PointXYZ(1., 1., 0.));
  cloud->push_back(pcl::PointXYZ(2., -3., 0.));
  cloud->push_back(pcl::PointXYZ(-1., 3., 0.));

  const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
  const Range<pcl::PointXYZ> range(ref_points);

  EXPECT_EQ(range(1, 4).size(), static_cast<std::uint32_t>(3));
  EXPECT_EQ(range(1, 3).size(), static_cast<std::uint32_t>(2));

  const std::vector<double> ranges = range(0, 4);
  for (unsigned int i = 0; i < cloud->size(); i++) {
    EXPECT_NEAR(ranges.at(i), norm(cloud->at(i)), 1e-7);
  }
}
