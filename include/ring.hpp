// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef RING_HPP_
#define RING_HPP_

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


bool RingIsAvailable(const std::vector<sensor_msgs::msg::PointField> & fields)
{
  for (const auto & field : fields) {
    if (field.name == "ring") {
      return true;
    }
  }
  return false;
}

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

#endif  // RING_HPP_
