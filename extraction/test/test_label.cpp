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

#include "lidar_feature_extraction/neighbor.hpp"
#include "lidar_feature_extraction/label.hpp"


TEST(Label, InitLabels)
{
  EXPECT_THAT(
    InitLabels(2),
    testing::ElementsAre(PointLabel::Default, PointLabel::Default));
}

bool equal(const PointXYZIR & p0, const PointXYZIR & p1)
{
  return
    p0.x == p1.x &&
    p0.y == p1.y &&
    p0.z == p1.z &&
    p0.intensity == p1.intensity &&
    p0.ring == p1.ring;
}

TEST(Extraction, AppendXYZIR)
{
  std::vector<PointXYZIR> points;
  points.push_back(PointXYZIR(0., 1., 2., 10., 0));
  points.push_back(PointXYZIR(3., 4., 5., 20., 1));
  points.push_back(PointXYZIR(6., 7., 8., 30., 2));
  points.push_back(PointXYZIR(2., 4., 6., 40., 3));

  const std::vector<double> curvature{0., 2., 4., 6.};

  pcl::PointCloud<PointXYZIR>::Ptr output_cloud(new pcl::PointCloud<PointXYZIR>());

  AppendXYZIR(output_cloud, points, curvature);

  EXPECT_EQ(output_cloud->size(), static_cast<size_t>(4));
  EXPECT_TRUE(equal(output_cloud->at(0), PointXYZIR(0., 1., 2., 0., 0)));
  EXPECT_TRUE(equal(output_cloud->at(1), PointXYZIR(3., 4., 5., 2., 1)));
  EXPECT_TRUE(equal(output_cloud->at(2), PointXYZIR(6., 7., 8., 4., 2)));
  EXPECT_TRUE(equal(output_cloud->at(3), PointXYZIR(2., 4., 6., 6., 3)));
}

TEST(Extraction, EdgeLabel)
{
  {
    const int padding = 2;
    const double threshold = 1.5;

    const EdgeLabel label(padding, threshold);

    std::vector<PointLabel> labels = InitLabels(8);
    const std::vector<double> curvature{3, 1, 2, 1, 1, 4, 1, 1};
    const NeighborCheckDebug is_neighbor({0, 0, 0, 0, 0, 0, 0, 0});
    label.Assign(labels, curvature, is_neighbor);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Edge,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::Edge,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor
      )
    );
  }

  {
    const int padding = 2;
    const double threshold = 1.5;

    const EdgeLabel label(padding, threshold);

    std::vector<PointLabel> labels = InitLabels(8);
    const std::vector<double> curvature{0, 2, 4, 1, 0, 2, 1, 1};
    const NeighborCheckDebug is_neighbor({0, 0, 0, 0, 1, 1, 1, 1});
    label.Assign(labels, curvature, is_neighbor);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::Edge,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::Edge,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor
      )
    );
  }
}
