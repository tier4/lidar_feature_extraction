// Copyright 2022 Tixiao Shan, Takeshi Ishita
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Tixiao Shan, Takeshi Ishita nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <gmock/gmock.h>

#include <iterator>

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
  struct Point
  {
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

TEST(Ring, RemoveSparseRings)
{
  {
    std::unordered_map<int, std::vector<int>> rings;
    rings[0] = {0, 1, 2};
    rings[2] = {0, 1, 2, 3};
    rings[4] = {0, 1};
    rings[6] = {0, 1, 2};
    RemoveSparseRings(rings, 3);

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
    RemoveSparseRings(rings, 4);

    EXPECT_THAT(rings.size(), 1);
    EXPECT_THAT(rings.at(2).size(), 4);
  }
}

TEST(Ring, ExtractAngleSortedRings) {
  struct PointWithRing
  {
    int ring;
    double x;
    double y;
  };

  std::vector<PointWithRing> points;
  points.push_back(PointWithRing{0, 1., 1.});
  points.push_back(PointWithRing{0, 1., 0.});

  points.push_back(PointWithRing{1, 1., -1.});
  points.push_back(PointWithRing{1, 1., 0.});
  points.push_back(PointWithRing{1, 0., 1.});

  points.push_back(PointWithRing{2, 1., 1.});
  points.push_back(PointWithRing{2, 0., -1.});
  points.push_back(PointWithRing{2, -1., -1.});

  const auto rings = ExtractAngleSortedRings(points);

  EXPECT_THAT(rings.at(0), testing::ElementsAre(1, 0));
  EXPECT_THAT(rings.at(1), testing::ElementsAre(2, 3, 4));
  EXPECT_THAT(rings.at(2), testing::ElementsAre(7, 6, 5));
}
