// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "curvature_label.hpp"
#include "extraction.hpp"
#include "reference_wrapper.hpp"

TEST(Extraction, ExtractByLabel)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  input_cloud->push_back(pcl::PointXYZ{0., 0., 0.});
  input_cloud->push_back(pcl::PointXYZ{0., 0., 1.});
  input_cloud->push_back(pcl::PointXYZ{0., 1., 0.});
  input_cloud->push_back(pcl::PointXYZ{0., 1., 1.});

  ConstReferenceVector<pcl::PointXYZ> input_ref_points(input_cloud->begin(), input_cloud->end());

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
