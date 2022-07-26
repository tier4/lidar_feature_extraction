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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <utility>
#include <set>
#include <functional>
#include <iterator>
#include <unordered_map>
#include <vector>

#include "cloud_iterator.hpp"
#include "iterator.hpp"
#include "mapped_points.hpp"


bool RingIsAvailable(const std::vector<sensor_msgs::msg::PointField> & fields);

template<typename Iter>
void SortByAtan2(std::vector<int> & indices, const Iter & iter)
{
  typedef ElementType<Iter> Element;

  auto f = [&](const int & index1, const int & index2) {
      const Element & p1 = iter.at(index1);
      const Element & p2 = iter.at(index2);
      const double angle1 = std::atan2(p1.y, p1.x);
      const double angle2 = std::atan2(p2.y, p2.x);
      return angle1 < angle2;
    };

  std::sort(indices.begin(), indices.end(), f);
}

template<typename Iter>
std::unordered_map<int, std::vector<int>>
MakeReferenceVectorsPerRing(const Iter & points)
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
  auto point_indices = MakeReferenceVectorsPerRing(iterator);
  SortEachRingByAngle<Iter>(point_indices, iterator);
  return point_indices;
}

#endif  // LIDAR_FEATURE_EXTRACTION__RING_HPP_
