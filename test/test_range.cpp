// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "range.hpp"

template<typename T>
bool Equal(const T & a, const T & b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

TEST(Range, IsInInclusiveRange) {
  EXPECT_TRUE(IsInInclusiveRange(3., 1., 5.));
  EXPECT_TRUE(IsInInclusiveRange(1., 1., 5.));
  EXPECT_TRUE(IsInInclusiveRange(5., 1., 5.));

  EXPECT_FALSE(IsInInclusiveRange(0., 1., 5.));
  EXPECT_FALSE(IsInInclusiveRange(6., 1., 5.));
}

TEST(Range, FilterByRange) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(0., 0., 0));
  cloud.push_back(pcl::PointXYZ(0., 1., 0));
  cloud.push_back(pcl::PointXYZ(1., 0., 1));
  cloud.push_back(pcl::PointXYZ(2., 0., 0));
  cloud.push_back(pcl::PointXYZ(2., 2., 0));

  const pcl::PointCloud<pcl::PointXYZ> r = FilterByRange(cloud, 1., 2.);
  EXPECT_EQ(r.size(), static_cast<std::uint32_t>(3));
  EXPECT_TRUE(Equal(r.at(0), cloud.at(1)));
  EXPECT_TRUE(Equal(r.at(1), cloud.at(2)));
  EXPECT_TRUE(Equal(r.at(2), cloud.at(3)));
}

TEST(Range, Range) {
  auto norm = [](const pcl::PointXYZ & p) {
    return std::sqrt(p.x * p.x + p.y * p.y);
  };

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->push_back(pcl::PointXYZ(3., 4., 0.));
  cloud->push_back(pcl::PointXYZ(1., 1., 0.));
  cloud->push_back(pcl::PointXYZ(2., -3., 0.));
  cloud->push_back(pcl::PointXYZ(-1., 3., 0.));

  const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
  const Range<pcl::PointXYZ> range(ref_points);

  EXPECT_EQ(range(1, 4).size(), static_cast<std::uint32_t>(3));
  EXPECT_EQ(range(1, 3).size(), static_cast<std::uint32_t>(2));

  const std::vector<double> ranges = range(0, 4);
  for (unsigned int i = 0; i < cloud->size(); i++) {
    EXPECT_NEAR(ranges.at(i), norm(cloud->at(i)), 1e-7);
  }
}
