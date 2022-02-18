// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include "ring.hpp"

TEST(Ring, AngleFromX1Y0)
{
  struct Point {
    double x;
    double y;
  };

  auto to_vector = [](const Point & p) {
    return std::vector<double>{p.x, p.y};
  };

  std::vector<Point> points;
  points.push_back(Point{1., 1.});
  points.push_back(Point{1., 0.});
  points.push_back(Point{1., -1.});
  points.push_back(Point{0., 1.});
  points.push_back(Point{0., -1.});
  points.push_back(Point{-1., -1.});

  SortByAtan2(points);

  EXPECT_THAT(to_vector(points.at(0)), testing::ElementsAre(-1., -1.));
  EXPECT_THAT(to_vector(points.at(1)), testing::ElementsAre(0., -1.));
  EXPECT_THAT(to_vector(points.at(2)), testing::ElementsAre(1., -1.));
  EXPECT_THAT(to_vector(points.at(3)), testing::ElementsAre(1., 0.));
  EXPECT_THAT(to_vector(points.at(4)), testing::ElementsAre(1., 1.));
  EXPECT_THAT(to_vector(points.at(5)), testing::ElementsAre(0., 1.));
}
