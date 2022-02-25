// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "label.hpp"

TEST(Label, Label)
{
  const double edge_threshold = 1.0;
  const double surface_threshold = 0.1;
  const double radian_threshold = 0.1;
  const int padding = 2;
  const int offset = 2;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (unsigned int i = 0; i < 10; i++) {
    cloud.push_back(pcl::PointXYZ(1., 1., 0));
  }

  const std::vector<double> curvature{0.3, 0.2, 1.0, 0.2, 0.1, 0.3};

  {
    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud.size()));
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
    ASSERT_EQ(mask.Size(), static_cast<int>(cloud.size()));
    std::vector<CurvatureLabel> labels(mask.Size(), CurvatureLabel::Default);
    const Label<pcl::PointXYZ> label(curvature, padding, offset, edge_threshold, surface_threshold);
    label.Edge(labels, mask, 1);

    std::vector<CurvatureLabel> expected_labels(cloud.size(), CurvatureLabel::Default);
    expected_labels.at(4) = CurvatureLabel::Edge;

    ASSERT_EQ(mask.Size(), static_cast<int>(cloud.size()));
    for (unsigned int i = 0; i < cloud.size(); i++) {
      EXPECT_EQ(labels.at(i), expected_labels.at(i));
    }

    EXPECT_THAT(
      mask.Get(),
      testing::ElementsAre(false, false, true, true, true, true, true, false, false, false));
  }

  {
    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud.size()));
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
    std::vector<CurvatureLabel> labels(mask.Size(), CurvatureLabel::Default);

    const Label<pcl::PointXYZ> label(curvature, padding, offset, edge_threshold, surface_threshold);

    label.Surface(labels, mask);

    std::vector<CurvatureLabel> expected_labels(cloud.size(), CurvatureLabel::Default);
    expected_labels.at(6) = CurvatureLabel::Surface;

    ASSERT_EQ(mask.Size(), static_cast<int>(cloud.size()));
    for (unsigned int i = 0; i < cloud.size(); i++) {
      EXPECT_EQ(labels.at(i), expected_labels.at(i));
    }

    EXPECT_THAT(
      mask.Get(),
      testing::ElementsAre(false, false, false, false, true, true, true, true, true, false));
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
  const MappedPoints input_ref_points(*input_cloud, indices);

  auto to_vector = [](const pcl::PointXYZ & p) {
    return std::vector<double>{p.x, p.y, p.z};
  };

  {
    const std::vector<CurvatureLabel> labels {
      CurvatureLabel::Default,
      CurvatureLabel::Edge,
      CurvatureLabel::Default,
      CurvatureLabel::Edge
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    ExtractByLabel(output_cloud, input_ref_points, labels, CurvatureLabel::Edge);
    EXPECT_EQ(output_cloud->size(), static_cast<std::uint32_t>(2));
    EXPECT_THAT(to_vector(output_cloud->at(0)), testing::ElementsAre(0., 0., 1.));
    EXPECT_THAT(to_vector(output_cloud->at(1)), testing::ElementsAre(0., 1., 1.));
  }

  {
    const std::vector<CurvatureLabel> labels {
      CurvatureLabel::Surface,
      CurvatureLabel::Surface,
      CurvatureLabel::Default,
      CurvatureLabel::Default
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    ExtractByLabel(output_cloud, input_ref_points, labels, CurvatureLabel::Surface);
    EXPECT_EQ(output_cloud->size(), static_cast<std::uint32_t>(2));
    EXPECT_THAT(to_vector(output_cloud->at(0)), testing::ElementsAre(0., 0., 0.));
    EXPECT_THAT(to_vector(output_cloud->at(1)), testing::ElementsAre(0., 0., 1.));
  }
}
