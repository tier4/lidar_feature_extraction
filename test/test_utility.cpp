// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include "utility.hpp"

using PointField = sensor_msgs::msg::PointField;

TEST(Utility, ExtractSectionsByRing) {
  {
    pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>());
    const auto sections = ExtractSectionsByRing<PointXYZIR>(cloud);

    EXPECT_EQ(sections.size(), static_cast<std::uint32_t>(0));
  }

  {
    pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>());
    cloud->push_back(PointXYZIR{1., 0., 1., 0., 1., 3});
    cloud->push_back(PointXYZIR{2., 0., 1., 0., 1., 3});

    const auto sections = ExtractSectionsByRing<PointXYZIR>(cloud);

    EXPECT_EQ(sections.size(), static_cast<std::uint32_t>(1));

    EXPECT_EQ(sections.at(0).first, cloud->begin() + 0);
    EXPECT_EQ(sections.at(0).second, cloud->begin() + 2);

    const auto [begin, end] = sections.at(0);
    std::vector<PointXYZIR> points(begin, end);
    EXPECT_EQ(points.at(0).x, 1.);
    EXPECT_EQ(points.at(1).x, 2.);
  }

  {
    pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>());
    cloud->push_back(PointXYZIR{0., 0., 1., 0., 1., 0});
    cloud->push_back(PointXYZIR{0., 1., 1., 0., 1., 0});
    cloud->push_back(PointXYZIR{0., 0., 2., 0., 1., 1});
    cloud->push_back(PointXYZIR{0., 1., 2., 0., 1., 1});
    cloud->push_back(PointXYZIR{0., 0., 3., 0., 1., 2});
    cloud->push_back(PointXYZIR{0., 1., 3., 0., 1., 2});
    cloud->push_back(PointXYZIR{1., 1., 3., 0., 1., 2});

    const auto sections = ExtractSectionsByRing<PointXYZIR>(cloud);

    EXPECT_EQ(sections.size(), static_cast<std::uint32_t>(3));

    EXPECT_EQ(sections.at(0).first, cloud->begin() + 0);
    EXPECT_EQ(sections.at(0).second, cloud->begin() + 2);

    EXPECT_EQ(sections.at(1).first, cloud->begin() + 2);
    EXPECT_EQ(sections.at(1).second, cloud->begin() + 4);

    EXPECT_EQ(sections.at(2).first, cloud->begin() + 4);
    EXPECT_EQ(sections.at(2).second, cloud->begin() + 7);
  }

  {
    pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>());
    cloud->push_back(PointXYZIR{0., 0., 1., 0., 1., 0});
    cloud->push_back(PointXYZIR{0., 0., 2., 0., 1., 1});
    cloud->push_back(PointXYZIR{0., 1., 1., 0., 1., 2});
    cloud->push_back(PointXYZIR{0., 1., 2., 0., 1., 1});

    EXPECT_THROW(
      try {
        const auto sections = ExtractSectionsByRing<PointXYZIR>(cloud);
      } catch (const std::invalid_argument & e) {
        EXPECT_STREQ("Ring 1 has already appeared", e.what());
        throw e;
      },
      std::invalid_argument);
  }
}

TEST(Utility, Convolution1D) {
  {
    std::vector<double> input{1., -1, 2., 0., 1};
    std::vector<double> weight{1., 0., -1};

    const auto result = Convolution1D(input.begin(), input.end(), weight.begin(), weight.end());
    EXPECT_THAT(result, testing::ElementsAre(-1., -1., 1.));
  }

  {
    std::vector<double> input{1., -1, 2.};
    std::vector<double> weight{1., 0., -1};

    const auto result = Convolution1D(input.begin(), input.end(), weight.begin(), weight.end());
    EXPECT_THAT(result, testing::ElementsAre(-1.));
  }

  {
    std::vector<double> input{2., 0.};
    std::vector<double> weight{1., 0., -1};
    EXPECT_THROW(
      try {
        Convolution1D(input.begin(), input.end(), weight.begin(), weight.end());
      } catch (const std::invalid_argument & e) {
        EXPECT_STREQ("Input array size 2 cannot be smaller than weight size 3", e.what());
        throw e;
      },
      std::invalid_argument);
  }
}

TEST(Utility, MakeWeight)
{
  EXPECT_THAT(MakeWeight(2), testing::ElementsAre(1., 1., -4., 1., 1.));
  EXPECT_THAT(MakeWeight(3), testing::ElementsAre(1., 1., 1., -6., 1., 1., 1.));
}

TEST(Utility, CalcCurvature)
{
  const std::vector<double> range{1., 1., 2., 0., 1., 1., 0.};
  const int padding = 2;
  const std::vector<double> result = CalcCurvature(range, padding);

  const double e0 = 1 * 1 + 1 * 1 + 2 * (-4) + 0 * 1 + 1 * 1;
  const double e1 = 1 * 1 + 2 * 1 + 0 * (-4) + 1 * 1 + 1 * 1;
  const double e2 = 2 * 1 + 0 * 1 + 1 * (-4) + 1 * 1 + 0 * 1;

  EXPECT_THAT(result, testing::ElementsAre(e0 * e0, e1 * e1, e2 * e2));
}

TEST(Utility, Curvature)
{
  const double edge_threshold = 0.3;
  const double surface_threshold = 0.2;

  const std::vector<double> curvature_values{0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  const Curvature curvature(curvature_values, edge_threshold, surface_threshold);

  ASSERT_THAT(curvature.Size(), curvature_values.size());

  std::vector<bool> is_edge(curvature.Size());
  for (int i = 0; i < curvature.Size(); i++) {
    is_edge[i] = curvature.IsEdge(i);
  }

  std::vector<bool> is_surface(curvature.Size());
  for (int i = 0; i < curvature.Size(); i++) {
    is_surface[i] = curvature.IsSurface(i);
  }

  EXPECT_THAT(is_edge, testing::ElementsAre(false, false, false, true, true, true));

  EXPECT_THAT(is_surface, testing::ElementsAre(true, true, true, false, false, false));
}

