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


#ifndef LIDAR_FEATURE_EXTRACTION__RING_HPP_
#define LIDAR_FEATURE_EXTRACTION__RING_HPP_

#include <fmt/core.h>
#include <pcl/point_cloud.h>

#include <algorithm>
#include <utility>
#include <set>
#include <functional>
#include <iterator>
#include <unordered_map>
#include <vector>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "cloud_iterator.hpp"
#include "iterator.hpp"
#include "mapped_points.hpp"


bool RingIsAvailable(const std::vector<sensor_msgs::msg::PointField> & fields);

template<typename T>
struct AHasSmallerPolarAngleThanB
{
  bool operator()(const T & a, const T & b) const
  {
    // faster way to compute std::atan2(a.y, a.x) < std::atan2(b.y, b.x)

    // both have the same values,
    // including the case that a = {0, 0}, b = {0, 0}
    if (a.x == b.x && a.y == b.y) {
      return false;
    }

    const double lena = a.x * a.x + a.y * a.y;
    const double lenb = b.x * b.x + b.y * b.y;

    if (lena == 0) {
      if (b.y == 0) {
        // for the case that b.x > 0
        // --> atan2(b.x, b.y) == atan2(a.y, a.x) == 0
        return b.x < 0;
      }
      return b.y > 0;
    }

    if (lenb == 0) {
      return a.y < 0;
    }

    if (a.y == 0) {
      return (a.x >= 0) && (b.y >= 0);
    }

    if (b.y == 0) {
      return !((b.x >= 0) && (a.y >= 0));
    }

    // both have the same y sign
    if (a.y * b.y > 0) {
      const double det = a.x * b.y - a.y * b.x;
      return det > 0;
    }

    return a.y < 0;
  }
};

template<typename Iter>
void SortByAtan2(std::vector<int> & indices, const Iter & iter)
{
  typedef ElementType<Iter> Element;
  const AHasSmallerPolarAngleThanB<Element> comparator;

  auto f = [&](const int & index1, const int & index2) {
      return comparator(iter.at(index1), iter.at(index2));
    };

  std::sort(indices.begin(), indices.end(), f);
}

template<typename Iter>
std::unordered_map<int, std::vector<int>> MakePointIndices(const Iter & points)
{
  typedef ElementType<Iter> Element;

  std::unordered_map<int, std::vector<int>> point_indices;
  for (unsigned int i = 0; i < points.size(); i++) {
    const Element & p = points.at(i);
    point_indices[p.ring].push_back(i);
  }
  return point_indices;
}

void RemoveSparseRings(
  std::unordered_map<int, std::vector<int>> & rings,
  const int n_min_points);

template<typename Iter>
void SortEachRingByAngle(
  std::unordered_map<int, std::vector<int>> & point_indices,
  const Iter & iterator)
{
  for (auto & [ring, indices] : point_indices) {
    SortByAtan2(indices, iterator);
  }
}

template<typename Iter>
std::unordered_map<int, std::vector<int>> ExtractAngleSortedRings(const Iter & iterator)
{
  auto point_indices = MakePointIndices(iterator);
  SortEachRingByAngle<Iter>(point_indices, iterator);
  return point_indices;
}

#endif  // LIDAR_FEATURE_EXTRACTION__RING_HPP_
