// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gtest/gtest.h>
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
