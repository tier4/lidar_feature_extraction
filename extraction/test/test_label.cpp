// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_feature_extraction/label.hpp"


TEST(Label, Label)
{
  const double edge_threshold = 1.0;
  const double surface_threshold = 0.1;
  const double radian_threshold = 0.1;
  const int padding = 2;
  const int offset = 2;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (unsigned int i = 0; i < 10; i++) {
    cloud->push_back(pcl::PointXYZ(1., 1., 0));
  }

  const std::vector<double> curvature{0.3, 0.2, 1.0, 0.2, 0.1, 0.3};
  const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
  const NeighborCheckXY is_neighbor(ref_points, radian_threshold);

  {
    std::vector<PointLabel> labels = InitLabels(ref_points.Size());
    ASSERT_EQ(labels.size(), cloud->size());
    const EdgeLabel<pcl::PointXYZ> edge_label(padding, edge_threshold, 1);
    edge_label.Assign(labels, is_neighbor, curvature, offset);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::Edge,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::Default));
  }

  {
    std::vector<PointLabel> labels = InitLabels(ref_points.Size());
    const SurfaceLabel<pcl::PointXYZ> surface_label(padding, surface_threshold);
    surface_label.Assign(labels, is_neighbor, curvature, offset);

    EXPECT_THAT(
      labels,
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::SurfaceNeighbor,
        PointLabel::SurfaceNeighbor,
        PointLabel::Surface,
        PointLabel::SurfaceNeighbor,
        PointLabel::SurfaceNeighbor,
        PointLabel::Default));
  }
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
