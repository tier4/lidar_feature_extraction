// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_feature_extraction/mapped_points.hpp"


TEST(MappedPoints, MappedPoints)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
  cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
  cloud->push_back(pcl::PointXYZ(0.0, 2.0, 0.0));
  cloud->push_back(pcl::PointXYZ(0.0, 3.0, 0.0));

  const std::vector<int> indices{2, 0, 1, 3, 0, 1};

  const MappedPoints<pcl::PointXYZ> mapped_points(cloud, indices);

  EXPECT_EQ(mapped_points.Size(), 6);

  EXPECT_EQ(mapped_points.At(0).y, 2.);
  EXPECT_EQ(mapped_points.At(1).y, 0.);
  EXPECT_EQ(mapped_points.At(2).y, 1.);
  EXPECT_EQ(mapped_points.At(3).y, 3.);
  EXPECT_EQ(mapped_points.At(4).y, 0.);
  EXPECT_EQ(mapped_points.At(5).y, 1.);

  const MappedPoints<pcl::PointXYZ> sliced = mapped_points.Slice(1, 4);
  EXPECT_EQ(sliced.Size(), 3);
  EXPECT_EQ(sliced.At(0).y, 0.);
  EXPECT_EQ(sliced.At(1).y, 1.);
  EXPECT_EQ(sliced.At(2).y, 3.);
}
