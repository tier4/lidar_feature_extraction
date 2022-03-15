// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_feature_extraction/label.hpp"
#include "lidar_feature_extraction/point_label.hpp"

TEST(Label, InitLabels)
{
  EXPECT_THAT(
    InitLabels(2),
    testing::ElementsAre(PointLabel::Default, PointLabel::Default));
}

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
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    std::vector<PointLabel> labels = InitLabels(ref_points.Size());

    FillFromLeft(labels, is_neighbor, 1, 4, PointLabel::Edge);

    EXPECT_THAT(
      labels,
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
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    std::vector<PointLabel> labels = InitLabels(ref_points.Size());

    FillFromLeft(labels, is_neighbor, 1, 5, PointLabel::Edge);

    EXPECT_THAT(
      labels,
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
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);
    auto labels = InitLabels(ref_points.Size());

    FillFromLeft(labels, is_neighbor, 1, 3, PointLabel::Default);

    EXPECT_THROW(
      try {
        FillFromLeft(labels, is_neighbor, 1, 4, PointLabel::Default);
      } catch(const std::invalid_argument & e) {
        EXPECT_STREQ("end_index (which is 4) > labels.size() (which is 3)", e.what());
        throw e;
      },
      std::invalid_argument
    );

    FillFromLeft(labels, is_neighbor, 0, 2, PointLabel::Default);
    EXPECT_THROW(
      try {
        FillFromLeft(labels, is_neighbor, -1, 2, PointLabel::Default);
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
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    std::vector<PointLabel> labels = InitLabels(ref_points.Size());

    FillFromRight(labels, is_neighbor, 1, 3, PointLabel::Edge);

    EXPECT_THAT(
      labels,
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
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    std::vector<PointLabel> labels = InitLabels(ref_points.Size());

    FillFromRight(labels, is_neighbor, 1, 4, PointLabel::Edge);

    EXPECT_THAT(
      labels,
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
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    std::vector<PointLabel> labels = InitLabels(ref_points.Size());

    FillFromRight(labels, is_neighbor, 1, 2, PointLabel::Default);
    EXPECT_THROW(
      try {
        FillFromRight(labels, is_neighbor, 1, 3, PointLabel::Default);
      } catch(const std::invalid_argument & e) {
        EXPECT_STREQ("end_index (which is 3) >= labels.size() (which is 3)", e.what());
        throw e;
      },
      std::invalid_argument
    );

    FillFromRight(labels, is_neighbor, -1, 2, PointLabel::Default);
    EXPECT_THROW(
      try {
        FillFromRight(labels, is_neighbor, -2, 2, PointLabel::Default);
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
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);
    auto labels = InitLabels(ref_points.Size());

    FillNeighbors(labels, is_neighbor, 3, 2, PointLabel::EdgeNeighbor);

    EXPECT_THAT(
      labels,
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
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    std::vector<PointLabel> labels = InitLabels(ref_points.Size());

    FillNeighbors(labels, is_neighbor, 3, 2, PointLabel::EdgeNeighbor);

    EXPECT_THAT(
      labels,
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
    const NeighborCheckXY<pcl::PointXYZ> is_neighbor(ref_points, radian_threshold);

    std::vector<PointLabel> labels = InitLabels(ref_points.Size());

    EXPECT_THROW(
      try {
        FillNeighbors(labels, is_neighbor, 7, 3, PointLabel::Default);
      } catch (const std::invalid_argument & e) {
        EXPECT_STREQ("index + padding (which is 10) >= labels.size() (which is 10)", e.what());
        throw e;
      },
      std::invalid_argument);

    FillNeighbors(labels, is_neighbor, 3, 3, PointLabel::Default);
    EXPECT_THROW(
      try {
        FillNeighbors(labels, is_neighbor, 2, 3, PointLabel::Default);
      } catch (const std::invalid_argument & e) {
        EXPECT_STREQ("index - padding (which is -1) < 0 (which is 0)", e.what());
        throw e;
      },
      std::invalid_argument);
  }
}
