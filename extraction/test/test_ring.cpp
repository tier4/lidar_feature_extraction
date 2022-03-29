// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <iterator>

#include <gmock/gmock.h>

#include <range/v3/all.hpp>

#include "lidar_feature_extraction/ring.hpp"


TEST(Ring, RingIsAvailable)
{
  const std::vector<sensor_msgs::msg::PointField> with_ring = {
    sensor_msgs::msg::PointField()
      .set__name("intensity").set__offset(16).set__datatype(7).set__count(1),
    sensor_msgs::msg::PointField()
      .set__name("ring").set__offset(20).set__datatype(4).set__count(1)
  };
  EXPECT_TRUE(RingIsAvailable(with_ring));

  const std::vector<sensor_msgs::msg::PointField> without_ring = {
    sensor_msgs::msg::PointField()
      .set__name("intensity").set__offset(16).set__datatype(7).set__count(1)
  };
  EXPECT_FALSE(RingIsAvailable(without_ring));
}

TEST(Ring, SortByAtan2)
{
  struct Point {
    double x;
    double y;
  };

  std::vector<Point> points;
  points.push_back(Point{1., 1.});    //       pi / 4
  points.push_back(Point{1., 0.});    //            0
  points.push_back(Point{1., -1.});   // -     pi / 4
  points.push_back(Point{0., 1.});    //       pi / 2
  points.push_back(Point{0., -1.});   // -     pi / 2
  points.push_back(Point{-1., -1.});  // - 3 * pi / 4

  const int size = static_cast<int>(points.size());
  std::vector<int> indices = ranges::views::ints(0, size) | ranges::to_vector;
  SortByAtan2(indices, points);
  EXPECT_THAT(indices, testing::ElementsAre(5, 4, 2, 1, 0, 3));
}

TEST(Ring, RemoveInsufficientNumRing)
{
  {
    std::unordered_map<int, std::vector<int>> rings;
    rings[0] = {0, 1, 2};
    rings[2] = {0, 1, 2, 3};
    rings[4] = {0, 1};
    rings[6] = {0, 1, 2};
    RemoveInsufficientNumRing(rings, 3);

    EXPECT_THAT(rings.size(), 3);
    EXPECT_THAT(rings.at(0).size(), 3);
    EXPECT_THAT(rings.at(2).size(), 4);
    EXPECT_THAT(rings.at(6).size(), 3);
  }

  {
    std::unordered_map<int, std::vector<int>> rings;
    rings[0] = {0, 1, 2};
    rings[2] = {0, 1, 2, 3};
    rings[4] = {0, 1};
    rings[6] = {0, 1, 2};
    RemoveInsufficientNumRing(rings, 4);

    EXPECT_THAT(rings.size(), 1);
    EXPECT_THAT(rings.at(2).size(), 4);
  }
}

TEST(Ring, ExtractAngleSortedRings) {
  struct PointWithRing {
    int ring;
    double x;
    double y;
  };

  std::vector<PointWithRing> points;
  points.push_back(PointWithRing{0,  1.,  1.});
  points.push_back(PointWithRing{0,  1.,  0.});

  points.push_back(PointWithRing{1,  1., -1.});
  points.push_back(PointWithRing{1,  1.,  0.});
  points.push_back(PointWithRing{1,  0.,  1.});

  points.push_back(PointWithRing{2,  1.,  1.});
  points.push_back(PointWithRing{2,  0., -1.});
  points.push_back(PointWithRing{2, -1., -1.});

  const auto rings = ExtractAngleSortedRings(points);

  EXPECT_THAT(rings.at(0), testing::ElementsAre(1, 0));
  EXPECT_THAT(rings.at(1), testing::ElementsAre(2, 3, 4));
  EXPECT_THAT(rings.at(2), testing::ElementsAre(7, 6, 5));
}
