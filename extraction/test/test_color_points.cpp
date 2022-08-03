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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "lidar_feature_extraction/label.hpp"
#include "lidar_feature_extraction/point_label.hpp"
#include "lidar_feature_extraction/color_points.hpp"

using testing::ElementsAre;

TEST(ColorPoints, ColorPointsByLabel)
{
  auto to_vector = [](const pcl::PointXYZRGB & p) {
      return std::vector<float>{
      static_cast<float>(p.x),
      static_cast<float>(p.y),
      static_cast<float>(p.z),
      static_cast<float>(p.r),
      static_cast<float>(p.g),
      static_cast<float>(p.b)
      };
    };

  std::vector<PointLabel> labels = InitLabels(6);
  labels.at(0) = PointLabel::Default;
  labels.at(1) = PointLabel::Edge;
  labels.at(2) = PointLabel::EdgeNeighbor;
  labels.at(3) = PointLabel::OutOfRange;
  labels.at(4) = PointLabel::Occluded;
  labels.at(5) = PointLabel::ParallelBeam;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->push_back(pcl::PointXYZ(0, 0, 0));
  cloud->push_back(pcl::PointXYZ(1, 0, 0));
  cloud->push_back(pcl::PointXYZ(2, 0, 0));
  cloud->push_back(pcl::PointXYZ(3, 0, 0));
  cloud->push_back(pcl::PointXYZ(4, 0, 0));
  cloud->push_back(pcl::PointXYZ(5, 0, 0));

  const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
  const auto colored = ColorPointsByLabel<pcl::PointXYZ>(ref_points, labels);

  EXPECT_THAT(to_vector(colored->at(0)), ElementsAre(0., 0., 0., 255., 255., 255.));
  EXPECT_THAT(to_vector(colored->at(1)), ElementsAre(1., 0., 0., 255., 0., 0.));
  EXPECT_THAT(to_vector(colored->at(2)), ElementsAre(2., 0., 0., 255., 63., 0.));
  EXPECT_THAT(to_vector(colored->at(3)), ElementsAre(3., 0., 0., 127., 127., 127.));
  EXPECT_THAT(to_vector(colored->at(4)), ElementsAre(4., 0., 0., 255., 0., 255.));
  EXPECT_THAT(to_vector(colored->at(5)), ElementsAre(5., 0., 0., 0., 255., 0.));
}
