// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include "utility.hpp"

using PointField = sensor_msgs::msg::PointField;

template<typename T>
bool Equal(const T & a, const T & b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
};

TEST(Utility, RingIsAvailable)
{
  const std::vector<sensor_msgs::msg::PointField> with_ring = {
    PointField().set__name("intensity").set__offset(16).set__datatype(7).set__count(1),
    PointField().set__name("ring").set__offset(20).set__datatype(4).set__count(1)
  };
  EXPECT_TRUE(RingIsAvailable(with_ring));

  const std::vector<sensor_msgs::msg::PointField> without_ring = {
    PointField().set__name("intensity").set__offset(16).set__datatype(7).set__count(1)
  };
  EXPECT_FALSE(RingIsAvailable(without_ring));
}

TEST(Utility, XYNorm)
{
  EXPECT_EQ(XYNorm(0., 0.), 0.);
  EXPECT_EQ(XYNorm(-1., 0.), 1.);
  EXPECT_EQ(XYNorm(3., 4.), 5.);
}

TEST(Utility, IsInInclusiveRange) {
  EXPECT_TRUE(IsInInclusiveRange(3., 1., 5.));
  EXPECT_TRUE(IsInInclusiveRange(1., 1., 5.));
  EXPECT_TRUE(IsInInclusiveRange(5., 1., 5.));

  EXPECT_FALSE(IsInInclusiveRange(0., 1., 5.));
  EXPECT_FALSE(IsInInclusiveRange(6., 1., 5.));
}

TEST(Utility, Range) {
  auto norm = [](const pcl::PointXYZ & p) {
    return std::sqrt(p.x * p.x + p.y * p.y);
  };

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(3., 4., 0.));
  cloud.push_back(pcl::PointXYZ(1., 1., 0.));
  cloud.push_back(pcl::PointXYZ(2., -3., 0.));
  cloud.push_back(pcl::PointXYZ(-1., 3., 0.));

  Range<pcl::PointXYZ> range(cloud.begin());
  EXPECT_EQ(range(0), norm(cloud.at(0)));
  EXPECT_EQ(range(0, 4).size(), static_cast<std::uint32_t>(4));

  const std::vector<double> ranges = range(0, 4);
  for (unsigned int i = 0; i < cloud.size(); i++) {
    EXPECT_NEAR(ranges.at(i), norm(cloud.at(i)), 1e-7);
  }
}

TEST(Utility, ExtractSectionsByRing) {
  {
    pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>());
    const auto sections = ExtractSectionsByRing<PointXYZIR>(cloud);

    EXPECT_EQ(sections.size(), static_cast<long unsigned int>(0));
  }

  {
    pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>());
    cloud->push_back(PointXYZIR{1., 0., 1., 0., 1., 3});
    cloud->push_back(PointXYZIR{2., 0., 1., 0., 1., 3});

    const auto sections = ExtractSectionsByRing<PointXYZIR>(cloud);

    EXPECT_EQ(sections.size(), static_cast<long unsigned int>(1));

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

    EXPECT_EQ(sections.size(), static_cast<long unsigned int>(3));

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

TEST(Utility, FillFromLeft)
{
  const double radian_threshold = 0.2;

  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

    Mask<pcl::PointXYZ> mask(cloud.begin(), cloud.end(), radian_threshold);
    mask.FillFromLeft(1, 4);

    EXPECT_THAT(mask.Get(), testing::ElementsAre(false, true, true, true, false));
  }

  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1.00, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(1.01, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(1.02, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(4.01, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(4.02, 1.0, 0.0));

    Mask<pcl::PointXYZ> mask(cloud.begin(), cloud.end(), radian_threshold);
    mask.FillFromLeft(1, 5);

    EXPECT_THAT(mask.Get(), testing::ElementsAre(false, true, true, false, false));
  }
}

TEST(Utility, FillFromRight)
{
  const double radian_threshold = 0.2;

  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

    Mask<pcl::PointXYZ> mask(cloud.begin(), cloud.end(), radian_threshold);
    mask.FillFromRight(1, 4);

    EXPECT_THAT(mask.Get(), testing::ElementsAre(false, true, true, true, false));
  }

  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(1.00, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(1.01, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(1.02, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(4.01, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(4.02, 1.0, 0.0));

    Mask<pcl::PointXYZ> mask(cloud.begin(), cloud.end(), radian_threshold);
    mask.FillFromRight(1, 5);

    EXPECT_THAT(mask.Get(), testing::ElementsAre(false, false, false, true, true));
  }
}

TEST(Utility, FillNeighbors)
{
  const double radian_threshold = 0.2;

  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

    Mask<pcl::PointXYZ> mask(cloud.begin(), cloud.end(), radian_threshold);
    mask.FillNeighbors(3, 2);

    EXPECT_THAT(mask.Get(), testing::ElementsAre(false, true, true, true, true, true));
  }

  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(2.00, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(2.01, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(4.00, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(4.01, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(4.02, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(4.03, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(6.01, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(6.02, 1.0, 0.0));

    Mask<pcl::PointXYZ> mask(cloud.begin(), cloud.end(), radian_threshold);
    mask.FillNeighbors(3, 2);

    EXPECT_THAT(
      mask.Get(),
      testing::ElementsAre(false, false, true, true, true, true, false, false));
  }
}

TEST(Utility, Argsort)
{
  {
    const std::vector<double> curvature{0.3, 0.2, 1.0, 0.2, 0.0, 0.1};
    const std::vector<int> indices = Argsort(curvature);
    EXPECT_THAT(indices, testing::ElementsAre(4, 5, 1, 3, 0, 2));
  }

  {
    const std::vector<double> curvature{0.0, 0.0, 0.0, 0.0, 0.0};
    const std::vector<int> indices = Argsort(curvature);
    EXPECT_THAT(indices, testing::ElementsAre(0, 1, 2, 3, 4));
  }
}

TEST(Utility, MaskOccludedPoints)
{
  const double radian_threshold = 0.2;
  const double distance_threshold = 2.0;

  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(4.00, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(4.01, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(8.02, 2.0, 0.0));  // occlusion
    cloud.push_back(pcl::PointXYZ(8.03, 2.0, 0.0));
    cloud.push_back(pcl::PointXYZ(8.04, 2.0, 0.0));
    cloud.push_back(pcl::PointXYZ(8.05, 8.0, 0.0));  // discontinuity
    cloud.push_back(pcl::PointXYZ(8.06, 8.0, 0.0));

    {
      Mask<pcl::PointXYZ> mask(cloud.begin(), cloud.end(), radian_threshold);
      const Neighbor<pcl::PointXYZ> neighbor(cloud.begin(), radian_threshold);
      const Range<pcl::PointXYZ> range(cloud.begin());
      MaskOccludedPoints<pcl::PointXYZ>(mask, neighbor, range, 1, distance_threshold);

      EXPECT_THAT(mask.Get(), testing::ElementsAre(false, false, true, true, false, false, false));
    }

    {
      Mask<pcl::PointXYZ> mask(cloud.begin(), cloud.end(), radian_threshold);
      const Neighbor<pcl::PointXYZ> neighbor(cloud.begin(), radian_threshold);
      const Range<pcl::PointXYZ> range(cloud.begin());
      MaskOccludedPoints<pcl::PointXYZ>(mask, neighbor, range, 3, distance_threshold);

      EXPECT_THAT(mask.Get(), testing::ElementsAre(false, false, true, true, true, false, false));
    }
  }

  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.push_back(pcl::PointXYZ(8.00, 8.0, 0.0));
    cloud.push_back(pcl::PointXYZ(8.01, 8.0, 0.0));  // discontinuity
    cloud.push_back(pcl::PointXYZ(8.02, 2.0, 0.0));
    cloud.push_back(pcl::PointXYZ(8.03, 2.0, 0.0));
    cloud.push_back(pcl::PointXYZ(8.04, 2.0, 0.0));  // occlusion
    cloud.push_back(pcl::PointXYZ(4.00, 1.0, 0.0));
    cloud.push_back(pcl::PointXYZ(4.01, 1.0, 0.0));

    {
      Mask<pcl::PointXYZ> mask(cloud.begin(), cloud.end(), radian_threshold);
      const Neighbor<pcl::PointXYZ> neighbor(cloud.begin(), radian_threshold);
      const Range<pcl::PointXYZ> range(cloud.begin());
      MaskOccludedPoints<pcl::PointXYZ>(mask, neighbor, range, 1, distance_threshold);
      EXPECT_THAT(mask.Get(), testing::ElementsAre(false, false, false, true, true, false, false));
    }

    {
      Mask<pcl::PointXYZ> mask(cloud.begin(), cloud.end(), radian_threshold);
      const Neighbor<pcl::PointXYZ> neighbor(cloud.begin(), radian_threshold);
      const Range<pcl::PointXYZ> range(cloud.begin());
      MaskOccludedPoints<pcl::PointXYZ>(mask, neighbor, range, 4, distance_threshold);
      EXPECT_THAT(mask.Get(), testing::ElementsAre(false, false, true, true, true, false, false));
    }
  }
}

TEST(Utility, IndexRange)
{
  {
    const IndexRange index_range(0, 12, 3);

    EXPECT_EQ(index_range.Begin(0), 0);
    EXPECT_EQ(index_range.End(0), 4);

    EXPECT_EQ(index_range.Begin(1), 4);
    EXPECT_EQ(index_range.End(1), 8);

    EXPECT_EQ(index_range.Begin(2), 8);
    EXPECT_EQ(index_range.End(2), 12);
  }

  {
    const IndexRange index_range(0, 14, 4);

    EXPECT_EQ(index_range.Begin(0), 0);
    EXPECT_EQ(index_range.End(0), 3);

    EXPECT_EQ(index_range.Begin(1), 3);
    EXPECT_EQ(index_range.End(1), 7);

    EXPECT_EQ(index_range.Begin(2), 7);
    EXPECT_EQ(index_range.End(2), 10);

    EXPECT_EQ(index_range.Begin(3), 10);
    EXPECT_EQ(index_range.End(3), 14);
  }

  {
    const IndexRange index_range(0, 12, 3);

    EXPECT_THROW(
      try {
        index_range.Begin(-1);
      } catch (std::out_of_range & e) {
        EXPECT_STREQ("j (which is -1) < 0 (which is 0)", e.what());
        throw e;
      },
      std::out_of_range);

    EXPECT_THROW(
      try {
        index_range.End(-1);
      } catch (std::out_of_range & e) {
        EXPECT_STREQ("j (which is -1) < 0 (which is 0)", e.what());
        throw e;
      },
      std::out_of_range);

    EXPECT_THROW(
      try {
        index_range.Begin(3);
      } catch (std::out_of_range & e) {
        EXPECT_STREQ("j (which is 3) >= n_blocks (which is 3)", e.what());
        throw e;
      },
      std::out_of_range);

    EXPECT_THROW(
      try {
        index_range.End(3);
      } catch (std::out_of_range & e) {
        EXPECT_STREQ("j (which is 3) >= n_blocks (which is 3)", e.what());
        throw e;
      },
      std::out_of_range);
  }
}

TEST(Utility, PaddedIndexRange) {
  {
    const PaddedIndexRange index_range(0, 17, 3, 1);

    EXPECT_EQ(index_range.Begin(0), 0);
    EXPECT_EQ(index_range.End(0), 7);

    EXPECT_EQ(index_range.Begin(1), 5);
    EXPECT_EQ(index_range.End(1), 12);

    EXPECT_EQ(index_range.Begin(2), 10);
    EXPECT_EQ(index_range.End(2), 17);
  }

  {
    const PaddedIndexRange index_range(1, 20, 3, 2);

    EXPECT_EQ(index_range.Begin(0), 1);
    EXPECT_EQ(index_range.End(0), 10);

    EXPECT_EQ(index_range.Begin(1), 6);
    EXPECT_EQ(index_range.End(1), 15);

    EXPECT_EQ(index_range.Begin(2), 11);
    EXPECT_EQ(index_range.End(2), 20);
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

TEST(Utility, FilterByRange) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0., 0., 0));
  cloud.push_back(pcl::PointXYZ(0., 1., 0));
  cloud.push_back(pcl::PointXYZ(1., 0., 1));
  cloud.push_back(pcl::PointXYZ(2., 0., 0));
  cloud.push_back(pcl::PointXYZ(2., 2., 0));

  const pcl::PointCloud<pcl::PointXYZ> r = FilterByRange(cloud, 1., 2.);
  EXPECT_EQ(r.size(), static_cast<long unsigned int>(3));
  EXPECT_TRUE(Equal(r.at(0), cloud.at(1)));
  EXPECT_TRUE(Equal(r.at(1), cloud.at(2)));
  EXPECT_TRUE(Equal(r.at(2), cloud.at(3)));
}

TEST(Utility, ExtractElements) {
  auto point_to_index = [](const pcl::PointXYZ & p) {
    return static_cast<int>(p.x + p.y + p.z);
  };

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0, 0, 0));
  cloud.push_back(pcl::PointXYZ(0, 1, 1));
  cloud.push_back(pcl::PointXYZ(1, 0, 2));
  cloud.push_back(pcl::PointXYZ(2, 0, 0));  // Duplicate index with cloud.at(1). Ignored
  cloud.push_back(pcl::PointXYZ(2, 2, 0));

  const auto result = ExtractElements<pcl::PointXYZ>(point_to_index, cloud);

  EXPECT_EQ(result.size(), static_cast<long unsigned int>(4));

  EXPECT_TRUE(result.find(0) != result.end());
  EXPECT_TRUE(result.find(2) != result.end());
  EXPECT_TRUE(result.find(3) != result.end());
  EXPECT_TRUE(result.find(4) != result.end());

  EXPECT_TRUE(Equal(result.at(0), cloud.at(0)));
  EXPECT_TRUE(Equal(result.at(2), cloud.at(1)));
  EXPECT_TRUE(Equal(result.at(3), cloud.at(2)));
  EXPECT_TRUE(Equal(result.at(4), cloud.at(4)));
}
