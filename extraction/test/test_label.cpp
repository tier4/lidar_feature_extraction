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

TEST(Extraction, ExtractByLabel)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  input_cloud->push_back(pcl::PointXYZ{0., 0., 0.});
  input_cloud->push_back(pcl::PointXYZ{0., 0., 1.});
  input_cloud->push_back(pcl::PointXYZ{0., 1., 0.});
  input_cloud->push_back(pcl::PointXYZ{0., 1., 1.});

  const std::vector<int> indices = irange(input_cloud->size());
  const MappedPoints<pcl::PointXYZ> input_ref_points(input_cloud, indices);

  auto to_vector = [](const pcl::PointXYZ & p) {
    return std::vector<double>{p.x, p.y, p.z};
  };

  {
    std::vector<PointLabel> labels = InitLabels(input_ref_points.Size());
    labels.at(0) = PointLabel::Default;
    labels.at(1) = PointLabel::Edge;
    labels.at(2) = PointLabel::Default;
    labels.at(3) = PointLabel::Edge;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    ExtractByLabel(output_cloud, input_ref_points, labels, PointLabel::Edge);
    EXPECT_EQ(output_cloud->size(), static_cast<std::uint32_t>(2));
    EXPECT_THAT(to_vector(output_cloud->at(0)), testing::ElementsAre(0., 0., 1.));
    EXPECT_THAT(to_vector(output_cloud->at(1)), testing::ElementsAre(0., 1., 1.));
  }

  {
    std::vector<PointLabel> labels = InitLabels(input_ref_points.Size());
    labels.at(0) = PointLabel::Surface;
    labels.at(1) = PointLabel::Surface;
    labels.at(2) = PointLabel::Default;
    labels.at(3) = PointLabel::Default;

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    ExtractByLabel(output_cloud, input_ref_points, labels, PointLabel::Surface);
    EXPECT_EQ(output_cloud->size(), static_cast<std::uint32_t>(2));
    EXPECT_THAT(to_vector(output_cloud->at(0)), testing::ElementsAre(0., 0., 0.));
    EXPECT_THAT(to_vector(output_cloud->at(1)), testing::ElementsAre(0., 0., 1.));
  }
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

TEST(Extraction, SurfaceLabel)
{
  {
    const int padding = 2;
    const double threshold = 0.5;

    const SurfaceLabel label(padding, threshold);

    std::vector<PointLabel> labels = InitLabels(8);
    const std::vector<double> curvature{0, 1, 1, 0, 2, 0, 1, 1};
    const NeighborCheckDebug is_neighbor({0, 0, 0, 0, 0, 0, 0, 0});
    label.Assign(labels, curvature, is_neighbor);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Surface,
        PointLabel::SurfaceNeighbor,
        PointLabel::SurfaceNeighbor,
        PointLabel::Surface,
        PointLabel::SurfaceNeighbor,
        PointLabel::SurfaceNeighbor,
        PointLabel::Default,
        PointLabel::Default
      )
    );
  }

  {
    const int padding = 2;
    const double threshold = 0.5;

    const SurfaceLabel label(padding, threshold);

    std::vector<PointLabel> labels = InitLabels(8);
    const std::vector<double> curvature{1, 1, 1, 0, 1, 1, 0, 1};
    const NeighborCheckDebug is_neighbor({0, 0, 1, 1, 1, 0, 0, 0});
    label.Assign(labels, curvature, is_neighbor);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::SurfaceNeighbor,
        PointLabel::Surface,
        PointLabel::SurfaceNeighbor,
        PointLabel::SurfaceNeighbor,
        PointLabel::Surface,
        PointLabel::SurfaceNeighbor
      )
    );
  }
}
