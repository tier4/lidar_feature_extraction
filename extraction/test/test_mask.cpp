// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mask.hpp"

TEST(Mask, FillFromLeft)
{
  const double radian_threshold = 0.2;

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
    mask.FillFromLeft(1, 4);

    EXPECT_THAT(mask.Get(), testing::ElementsAre(false, true, true, true, false));
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(1.00, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(1.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(1.02, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.02, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
    mask.FillFromLeft(1, 5);

    EXPECT_THAT(mask.Get(), testing::ElementsAre(false, true, true, false, false));
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);

    mask.FillFromLeft(1, 3);
    EXPECT_THROW(
      try {
        mask.FillFromLeft(1, 4);
      } catch(const std::invalid_argument & e) {
        EXPECT_STREQ("end_index (which is 4) > this->Size() (which is 3)", e.what());
        throw e;
      },
      std::invalid_argument
    );

    mask.FillFromLeft(0, 2);
    EXPECT_THROW(
      try {
        mask.FillFromLeft(-1, 2);
      } catch(const std::invalid_argument & e) {
        EXPECT_STREQ("begin_index (which is -1) < 0 (which is 0)", e.what());
        throw e;
      },
      std::invalid_argument
    );
  }
}

TEST(Mask, FillFromRight)
{
  const double radian_threshold = 0.2;

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
    mask.FillFromRight(1, 3);

    EXPECT_THAT(mask.Get(), testing::ElementsAre(false, false, true, true, false));
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(1.00, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(1.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(1.02, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.02, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
    mask.FillFromRight(1, 4);

    EXPECT_THAT(mask.Get(), testing::ElementsAre(false, false, false, true, true));
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);

    mask.FillFromRight(1, 2);
    EXPECT_THROW(
      try {
        mask.FillFromRight(1, 3);
      } catch(const std::invalid_argument & e) {
        EXPECT_STREQ("end_index (which is 3) >= this->Size() (which is 3)", e.what());
        throw e;
      },
      std::invalid_argument
    );

    mask.FillFromRight(-1, 2);
    EXPECT_THROW(
      try {
        mask.FillFromRight(-2, 2);
      } catch(const std::invalid_argument & e) {
        EXPECT_STREQ("begin_index (which is -2) < -1 (which is -1)", e.what());
        throw e;
      },
      std::invalid_argument
    );
  }
}

TEST(Mask, FillNeighbors)
{
  const double radian_threshold = 0.2;

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
    mask.FillNeighbors(3, 2);

    EXPECT_THAT(mask.Get(), testing::ElementsAre(false, true, true, true, true, true));
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(2.00, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(2.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.00, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.02, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.03, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(6.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(6.02, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
    mask.FillNeighbors(3, 2);

    EXPECT_THAT(
      mask.Get(),
      testing::ElementsAre(false, false, true, true, true, true, false, false));
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (unsigned int i = 0; i < 10; i++) {
      cloud->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));
    }

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
    EXPECT_THROW(
      try {
        mask.FillNeighbors(7, 3);
      } catch (const std::invalid_argument & e) {
        EXPECT_STREQ("index + padding (which is 10) >= this->Size() (which is 10)", e.what());
        throw e;
      },
      std::invalid_argument);

    mask.FillNeighbors(3, 3);
    EXPECT_THROW(
      try {
        mask.FillNeighbors(2, 3);
      } catch (const std::invalid_argument & e) {
        EXPECT_STREQ("index - padding (which is -1) < 0 (which is 0)", e.what());
        throw e;
      },
      std::invalid_argument);
  }
}

TEST(Mask, MaskOccludedPoints)
{
  const double radian_threshold = 0.2;
  const double distance_threshold = 2.0;

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(4.00, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.02, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.03, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.04, 2.0, 0.0));  // occlusion
    cloud->push_back(pcl::PointXYZ(8.05, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.06, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.07, 8.0, 0.0));  // discontinuity
    cloud->push_back(pcl::PointXYZ(8.08, 8.0, 0.0));

    {
      const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
      Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
      const Neighbor<pcl::PointXYZ> neighbor(ref_points, radian_threshold);
      const Range<pcl::PointXYZ> range(ref_points);
      MaskOccludedPoints<pcl::PointXYZ>(mask, neighbor, range, 1, distance_threshold);

      EXPECT_THAT(
        mask.Get(),
        testing::ElementsAre(false, false, false, false, true, true, false, false, false));
    }

    {
      const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
      Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
      const Neighbor<pcl::PointXYZ> neighbor(ref_points, radian_threshold);
      const Range<pcl::PointXYZ> range(ref_points);
      MaskOccludedPoints<pcl::PointXYZ>(mask, neighbor, range, 3, distance_threshold);

      EXPECT_THAT(
        mask.Get(),
        testing::ElementsAre(false, false, false, false, true, true, true, false, false));
    }
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(8.03, 8.0, 0.0));  // discontinuity
    cloud->push_back(pcl::PointXYZ(8.04, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.05, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.06, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.07, 2.0, 0.0));
    cloud->push_back(pcl::PointXYZ(8.08, 2.0, 0.0));  // occlusion
    cloud->push_back(pcl::PointXYZ(4.09, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.10, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.11, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.12, 1.0, 0.0));

    {
      const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
      Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
      const Neighbor<pcl::PointXYZ> neighbor(ref_points, radian_threshold);
      const Range<pcl::PointXYZ> range(ref_points);
      MaskOccludedPoints<pcl::PointXYZ>(mask, neighbor, range, 1, distance_threshold);
      EXPECT_THAT(
        mask.Get(),
        testing::ElementsAre(false, false, false, false, true, true, false, false, false, false));
    }

    {
      const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
      Mask<pcl::PointXYZ> mask(ref_points, radian_threshold);
      const Neighbor<pcl::PointXYZ> neighbor(ref_points, radian_threshold);
      const Range<pcl::PointXYZ> range(ref_points);
      MaskOccludedPoints<pcl::PointXYZ>(mask, neighbor, range, 3, distance_threshold);
      EXPECT_THAT(
        mask.Get(),
        testing::ElementsAre(false, false, true, true, true, true, false, false, false, false));
    }
  }
}

