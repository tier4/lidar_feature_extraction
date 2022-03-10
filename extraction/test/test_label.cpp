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

  {
    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    Label<pcl::PointXYZ> label(ref_points, radian_threshold);
    ASSERT_EQ(label.Size(), static_cast<int>(cloud->size()));
    const EdgeLabel<pcl::PointXYZ> edge_label(padding, edge_threshold, 1);
    edge_label.Assign(label, curvature, offset);

    EXPECT_THAT(
      label.Get(),
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
    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    Label<pcl::PointXYZ> label(ref_points, radian_threshold);

    const SurfaceLabel<pcl::PointXYZ> surface_label(padding, surface_threshold);
    surface_label.Assign(label, curvature, offset);

    EXPECT_THAT(
      label.Get(),
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
  const double radian_threshold = 0.2;

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
    Label<pcl::PointXYZ> label(input_ref_points, radian_threshold);
    label.Fill(0, PointLabel::Default);
    label.Fill(1, PointLabel::Edge);
    label.Fill(2, PointLabel::Default);
    label.Fill(3, PointLabel::Edge);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    ExtractByLabel(output_cloud, input_ref_points, label, PointLabel::Edge);
    EXPECT_EQ(output_cloud->size(), static_cast<std::uint32_t>(2));
    EXPECT_THAT(to_vector(output_cloud->at(0)), testing::ElementsAre(0., 0., 1.));
    EXPECT_THAT(to_vector(output_cloud->at(1)), testing::ElementsAre(0., 1., 1.));
  }

  {
    Label<pcl::PointXYZ> label(input_ref_points, radian_threshold);
    label.Fill(0, PointLabel::Surface);
    label.Fill(1, PointLabel::Surface);
    label.Fill(2, PointLabel::Default);
    label.Fill(3, PointLabel::Default);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    ExtractByLabel(output_cloud, input_ref_points, label, PointLabel::Surface);
    EXPECT_EQ(output_cloud->size(), static_cast<std::uint32_t>(2));
    EXPECT_THAT(to_vector(output_cloud->at(0)), testing::ElementsAre(0., 0., 0.));
    EXPECT_THAT(to_vector(output_cloud->at(1)), testing::ElementsAre(0., 0., 1.));
  }
}
