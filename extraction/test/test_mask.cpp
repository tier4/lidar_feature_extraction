// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_feature_extraction/label.hpp"
#include "lidar_feature_extraction/point_label.hpp"


TEST(Label, FillFromLeft)
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
    const NeighborCheck<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    Label label(is_neighbor);
    label.FillFromLeft(1, 4, PointLabel::Edge);

    EXPECT_THAT(
      label.Get(),
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Edge,
        PointLabel::Edge,
        PointLabel::Edge,
        PointLabel::Default));
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(1.00, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(1.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(1.02, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.02, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    const NeighborCheck<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    Label label(is_neighbor);
    label.FillFromLeft(1, 5, PointLabel::Edge);

    EXPECT_THAT(
      label.Get(),
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Edge,
        PointLabel::Edge,
        PointLabel::Default,
        PointLabel::Default));
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    const NeighborCheck<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    Label<pcl::PointXYZ> label(is_neighbor);

    label.FillFromLeft(1, 3, PointLabel::Default);
    EXPECT_THROW(
      try {
        label.FillFromLeft(1, 4, PointLabel::Default);
      } catch(const std::invalid_argument & e) {
        EXPECT_STREQ("end_index (which is 4) > this->Size() (which is 3)", e.what());
        throw e;
      },
      std::invalid_argument
    );

    label.FillFromLeft(0, 2, PointLabel::Default);
    EXPECT_THROW(
      try {
        label.FillFromLeft(-1, 2, PointLabel::Default);
      } catch(const std::invalid_argument & e) {
        EXPECT_STREQ("begin_index (which is -1) < 0 (which is 0)", e.what());
        throw e;
      },
      std::invalid_argument
    );
  }
}

TEST(Label, FillFromRight)
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
    const NeighborCheck<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    Label<pcl::PointXYZ> label(is_neighbor);
    label.FillFromRight(1, 3, PointLabel::Edge);

    EXPECT_THAT(
      label.Get(),
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::Edge,
        PointLabel::Edge,
        PointLabel::Default));
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(1.00, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(1.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(1.02, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.01, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(4.02, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    const NeighborCheck<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    Label<pcl::PointXYZ> label(is_neighbor);
    label.FillFromRight(1, 4, PointLabel::Edge);

    EXPECT_THAT(
      label.Get(),
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::Edge,
        PointLabel::Edge));
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
    cloud->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    const NeighborCheck<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    Label<pcl::PointXYZ> label(is_neighbor);

    label.FillFromRight(1, 2, PointLabel::Default);
    EXPECT_THROW(
      try {
        label.FillFromRight(1, 3, PointLabel::Default);
      } catch(const std::invalid_argument & e) {
        EXPECT_STREQ("end_index (which is 3) >= this->Size() (which is 3)", e.what());
        throw e;
      },
      std::invalid_argument
    );

    label.FillFromRight(-1, 2, PointLabel::Default);
    EXPECT_THROW(
      try {
        label.FillFromRight(-2, 2, PointLabel::Default);
      } catch(const std::invalid_argument & e) {
        EXPECT_STREQ("begin_index (which is -2) < -1 (which is -1)", e.what());
        throw e;
      },
      std::invalid_argument
    );
  }
}

TEST(Label, FillNeighbors)
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
    const NeighborCheck<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    Label<pcl::PointXYZ> label(is_neighbor);
    label.FillNeighbors(3, 2, PointLabel::EdgeNeighbor);

    EXPECT_THAT(
      label.Get(),
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor));
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
    const NeighborCheck<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    Label<pcl::PointXYZ> label(is_neighbor);
    label.FillNeighbors(3, 2, PointLabel::EdgeNeighbor);

    EXPECT_THAT(
      label.Get(),
      testing::ElementsAre(
        PointLabel::Default,
        PointLabel::Default,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::EdgeNeighbor,
        PointLabel::Default,
        PointLabel::Default));
  }

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (unsigned int i = 0; i < 10; i++) {
      cloud->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));
    }

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    const NeighborCheck<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    Label<pcl::PointXYZ> label(is_neighbor);
    EXPECT_THROW(
      try {
        label.FillNeighbors(7, 3, PointLabel::Default);
      } catch (const std::invalid_argument & e) {
        EXPECT_STREQ("index + padding (which is 10) >= this->Size() (which is 10)", e.what());
        throw e;
      },
      std::invalid_argument);

    label.FillNeighbors(3, 3, PointLabel::Default);
    EXPECT_THROW(
      try {
        label.FillNeighbors(2, 3, PointLabel::Default);
      } catch (const std::invalid_argument & e) {
        EXPECT_STREQ("index - padding (which is -1) < 0 (which is 0)", e.what());
        throw e;
      },
      std::invalid_argument);
  }
}

TEST(Label, LabelOutOfRange)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->push_back(pcl::PointXYZ(1.9, 0.0, 0.0));
  cloud->push_back(pcl::PointXYZ(2.0, 0.0, 0.0));
  cloud->push_back(pcl::PointXYZ(0.0, 5.0, 0.0));
  cloud->push_back(pcl::PointXYZ(0.0, 8.0, 0.0));
  cloud->push_back(pcl::PointXYZ(0.0, 8.1, 0.0));

  const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
  const Range<pcl::PointXYZ> range(ref_points);

  LabelBase label(ref_points.Size());
  LabelOutOfRange(label, range, 2.0, 8.0);
  EXPECT_THAT(
    label.Get(),
    testing::ElementsAre(
      PointLabel::OutOfRange,
      PointLabel::Default,
      PointLabel::Default,
      PointLabel::Default,
      PointLabel::OutOfRange));
}

TEST(Label, LabelOccludedPoints)
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

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    const NeighborCheck<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);
    const Range<pcl::PointXYZ> range(ref_points);

    {
      Label<pcl::PointXYZ> label(is_neighbor);
      LabelOccludedPoints<pcl::PointXYZ>(label, is_neighbor, range, 1, distance_threshold);

      EXPECT_THAT(
        label.Get(),
        testing::ElementsAre(
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default));
    }

    {
      Label<pcl::PointXYZ> label(is_neighbor);
      LabelOccludedPoints<pcl::PointXYZ>(label, is_neighbor, range, 3, distance_threshold);

      EXPECT_THAT(
        label.Get(),
        testing::ElementsAre(
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Default,
          PointLabel::Default));
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

    const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
    const NeighborCheck<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);
    const Range<pcl::PointXYZ> range(ref_points);

    {
      Label<pcl::PointXYZ> label(is_neighbor);
      LabelOccludedPoints<pcl::PointXYZ>(label, is_neighbor, range, 1, distance_threshold);
      EXPECT_THAT(
        label.Get(),
        testing::ElementsAre(
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default));
    }

    {
      Label<pcl::PointXYZ> label(is_neighbor);
      LabelOccludedPoints<pcl::PointXYZ>(label, is_neighbor, range, 3, distance_threshold);
      EXPECT_THAT(
        label.Get(),
        testing::ElementsAre(
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Occluded,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default,
          PointLabel::Default));
    }
  }
}

