// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mask.hpp"

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

    ConstReferenceVector<pcl::PointXYZ> ref_points(cloud.begin(), cloud.end());
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
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

    ConstReferenceVector<pcl::PointXYZ> ref_points(cloud.begin(), cloud.end());
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
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

    ConstReferenceVector<pcl::PointXYZ> ref_points(cloud.begin(), cloud.end());
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
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

    ConstReferenceVector<pcl::PointXYZ> ref_points(cloud.begin(), cloud.end());
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
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

    ConstReferenceVector<pcl::PointXYZ> ref_points(cloud.begin(), cloud.end());
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
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

    ConstReferenceVector<pcl::PointXYZ> ref_points(cloud.begin(), cloud.end());
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
    mask.FillNeighbors(3, 2);

    EXPECT_THAT(
      mask.Get(),
      testing::ElementsAre(false, false, true, true, true, true, false, false));
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
      ConstReferenceVector<pcl::PointXYZ> ref_points(cloud.begin(), cloud.end());
      Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
      const Neighbor<pcl::PointXYZ> neighbor(ref_points, radian_threshold);
      const Range<pcl::PointXYZ> range(ref_points);
      MaskOccludedPoints<pcl::PointXYZ>(mask, neighbor, range, 1, distance_threshold);

      EXPECT_THAT(mask.Get(), testing::ElementsAre(false, false, true, true, false, false, false));
    }

    {
      ConstReferenceVector<pcl::PointXYZ> ref_points(cloud.begin(), cloud.end());
      Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
      const Neighbor<pcl::PointXYZ> neighbor(ref_points, radian_threshold);
      const Range<pcl::PointXYZ> range(ref_points);
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
      ConstReferenceVector<pcl::PointXYZ> ref_points(cloud.begin(), cloud.end());
      Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
      const Neighbor<pcl::PointXYZ> neighbor(ref_points, radian_threshold);
      const Range<pcl::PointXYZ> range(ref_points);
      MaskOccludedPoints<pcl::PointXYZ>(mask, neighbor, range, 1, distance_threshold);
      EXPECT_THAT(mask.Get(), testing::ElementsAre(false, false, false, true, true, false, false));
    }

    {
      ConstReferenceVector<pcl::PointXYZ> ref_points(cloud.begin(), cloud.end());
      Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
      const Neighbor<pcl::PointXYZ> neighbor(ref_points, radian_threshold);
      const Range<pcl::PointXYZ> range(ref_points);
      MaskOccludedPoints<pcl::PointXYZ>(mask, neighbor, range, 4, distance_threshold);
      EXPECT_THAT(mask.Get(), testing::ElementsAre(false, false, true, true, true, false, false));
    }
  }
}

