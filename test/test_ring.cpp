// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <iterator>

#include <gmock/gmock.h>

#include "ring.hpp"

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
  points.push_back(Point{1., 1.});
  points.push_back(Point{1., 0.});
  points.push_back(Point{1., -1.});
  points.push_back(Point{0., 1.});
  points.push_back(Point{0., -1.});
  points.push_back(Point{-1., -1.});

  std::vector<std::reference_wrapper<const Point>> point_refs(points.begin(), points.end());
  SortByAtan2<Point>(point_refs);

  auto to_vector = [](const std::reference_wrapper<const Point> & ref_p) {
    const Point & p = ref_p.get();
    return std::vector<double>{p.x, p.y};
  };

  EXPECT_THAT(to_vector(point_refs.at(0)), testing::ElementsAre(-1., -1.));
  EXPECT_THAT(to_vector(point_refs.at(1)), testing::ElementsAre(0., -1.));
  EXPECT_THAT(to_vector(point_refs.at(2)), testing::ElementsAre(1., -1.));
  EXPECT_THAT(to_vector(point_refs.at(3)), testing::ElementsAre(1., 0.));
  EXPECT_THAT(to_vector(point_refs.at(4)), testing::ElementsAre(1., 1.));
  EXPECT_THAT(to_vector(point_refs.at(5)), testing::ElementsAre(0., 1.));
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

  auto get_xy = [](const PointWithRing & p) {
    return std::vector<double>{p.x, p.y};
  };

  const auto rings = ExtractAngleSortedRings(points);

  const auto ring0 = rings.at(0);
  EXPECT_THAT(get_xy(ring0.at(0)), testing::ElementsAre(1., 0.));
  EXPECT_THAT(get_xy(ring0.at(1)), testing::ElementsAre(1., 1.));

  const auto ring1 = rings.at(1);
  EXPECT_THAT(get_xy(ring1.at(0)), testing::ElementsAre(1., -1.));
  EXPECT_THAT(get_xy(ring1.at(1)), testing::ElementsAre(1.,  0.));
  EXPECT_THAT(get_xy(ring1.at(2)), testing::ElementsAre(0.,  1.));

  const auto ring2 = rings.at(2);
  EXPECT_THAT(get_xy(ring2.at(0)), testing::ElementsAre(-1.,  -1.));
  EXPECT_THAT(get_xy(ring2.at(1)), testing::ElementsAre( 0.,  -1.));
  EXPECT_THAT(get_xy(ring2.at(2)), testing::ElementsAre( 1.,   1.));
}
