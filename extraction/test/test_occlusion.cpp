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

#include "lidar_feature_extraction/occlusion.hpp"

const double neighbor_radian_threshold = 0.2;
const double distance_threshold = 2.0;


TEST(Label, FromLeft)
{
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(4.03, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.04, 2.0, 0.0));  // occlusion
    cloud->push_back(pcl::PointXYZ(8.05, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.06, 2.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, neighbor_radian_threshold);
    const Range<pcl::PointXYZ> range(ref_points);

    {
      std::vector<PointLabel> labels = InitLabels(ref_points.size());
      LabelOccludedPoints<pcl::PointXYZ>(labels, is_neighbor, range, 2, distance_threshold);

      EXPECT_THAT(
        labels,
        testing::ElementsAre(
          PointLabel::Default,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Occluded));
    }
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(4.00, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.02, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.03, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.04, 2.0, 0.0));  // occlusion
    cloud->push_back(pcl::PointXYZ(8.05, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.06, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.07, 8.0, 0.0));  // discontinuity
    cloud->push_back(pcl::PointXYZ(8.08, 8.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, neighbor_radian_threshold);
    const Range<pcl::PointXYZ> range(ref_points);

    {
      std::vector<PointLabel> labels = InitLabels(ref_points.size());
      LabelOccludedPoints<pcl::PointXYZ>(labels, is_neighbor, range, 1, distance_threshold);

      EXPECT_THAT(
        labels,
        testing::ElementsAre(
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default));
    }

    {
      std::vector<PointLabel> labels = InitLabels(ref_points.size());
      LabelOccludedPoints<pcl::PointXYZ>(labels, is_neighbor, range, 3, distance_threshold);

      EXPECT_THAT(
        labels,
        testing::ElementsAre(
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Default,
          PointLabel::Default));
    }
  }
}

TEST(Label, FromRight)
{
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(8.06, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.07, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.08, 2.0, 0.0));  // occlusion
    cloud->push_back(pcl::PointXYZ(4.09, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, neighbor_radian_threshold);
    const Range<pcl::PointXYZ> range(ref_points);

    {
      std::vector<PointLabel> labels = InitLabels(ref_points.size());
      LabelOccludedPoints<pcl::PointXYZ>(labels, is_neighbor, range, 2, distance_threshold);
      EXPECT_THAT(
        labels,
        testing::ElementsAre(
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Default));
    }
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(8.03, 8.0, 0.0));  // discontinuity
    cloud->push_back(pcl::PointXYZ(8.04, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.05, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.06, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.07, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.08, 2.0, 0.0));  // occlusion
    cloud->push_back(pcl::PointXYZ(4.09, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.10, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.11, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.12, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, neighbor_radian_threshold);
    const Range<pcl::PointXYZ> range(ref_points);

    {
      std::vector<PointLabel> labels = InitLabels(ref_points.size());
      LabelOccludedPoints<pcl::PointXYZ>(labels, is_neighbor, range, 1, distance_threshold);
      EXPECT_THAT(
        labels,
        testing::ElementsAre(
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default));
    }

    {
      std::vector<PointLabel> labels = InitLabels(ref_points.size());
      LabelOccludedPoints<pcl::PointXYZ>(labels, is_neighbor, range, 3, distance_threshold);
      EXPECT_THAT(
        labels,
        testing::ElementsAre(
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default));
    }
  }
}
