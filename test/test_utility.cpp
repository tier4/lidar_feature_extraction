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

TEST(Utility, ColumnIndex)
{
  EXPECT_EQ(ColumnIndex(100, 0, -1), 25);
  EXPECT_EQ(ColumnIndex(100, 1, 0), 50);
  EXPECT_EQ(ColumnIndex(100, 0, 1), 75);
  EXPECT_EQ(ColumnIndex(100, -1, 0), 100);
}

TEST(Utility, IsInInclusiveRange) {
  EXPECT_TRUE(IsInInclusiveRange(3., 1., 5.));
  EXPECT_TRUE(IsInInclusiveRange(1., 1., 5.));
  EXPECT_TRUE(IsInInclusiveRange(5., 1., 5.));

  EXPECT_FALSE(IsInInclusiveRange(0., 1., 5.));
  EXPECT_FALSE(IsInInclusiveRange(6., 1., 5.));
}

TEST(Utility, IndexRange) {
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
