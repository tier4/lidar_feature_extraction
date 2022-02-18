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
#include "reference_wrapper.hpp"


bool RingIsAvailable(const std::vector<sensor_msgs::msg::PointField> & fields)
{
  for (const auto & field : fields) {
    if (field.name == "ring") {
      return true;
    }
  }
  return false;
}

template<typename Element>
void SortByAtan2(ConstReferenceVector<Element> & iter)
{
  auto f = [](
    const ConstReferenceWrapper<Element> & ref_p1,
    const ConstReferenceWrapper<Element> & ref_p2) {
      const Element & p1 = ref_p1.get();
      const Element & p2 = ref_p2.get();
      const double angle1 = std::atan2(p1.y, p1.x);
      const double angle2 = std::atan2(p2.y, p2.x);
      return angle1 < angle2;
    };

  std::sort(iter.begin(), iter.end(), f);
}

template<typename Iter>
std::unordered_map<int, ConstReferenceVector<ElementType<Iter>>>
MakeReferenceVectorsPerRing(const Iter & points)
{
  typedef ElementType<Iter> Element;

  std::unordered_map<int, ConstReferenceVector<Element>> rings;

  for (const Element & p : points) {
    rings[p.ring].push_back(std::cref(p));
  }
  return rings;
}

template<typename Element>
void SortEachRingByAngle(std::unordered_map<int, ConstReferenceVector<Element>> & rings)
{
  for (auto & [ring, points] : rings) {
    SortByAtan2<Element>(points);
  }
}

template<typename Iter>
std::unordered_map<int, ConstReferenceVector<ElementType<Iter>>>
ExtractAngleSortedRings(const Iter & iterator)
{
  typedef ElementType<Iter> Element;

  auto rings = MakeReferenceVectorsPerRing(iterator);
  SortEachRingByAngle<Element>(rings);
  return rings;
}

#endif  // RING_HPP_
