// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _RING_LIDAR_ODOMETRY_H_
#define _RING_LIDAR_ODOMETRY_H_

#include <algorithm>
#include <utility>
#include <set>
#include <functional>
#include <iterator>
#include <unordered_map>
#include <vector>

#include <fmt/core.h>
#include <pcl/point_cloud.h>

#include "cloud_iterator.hpp"

template<typename Iterator>
using ElementType = typename std::iterator_traits<typename Iterator::iterator>::value_type;

template<typename Element>
using ConstReferenceVector = std::vector<std::reference_wrapper<const Element>>;

template<typename Element>
void SortByAtan2(ConstReferenceVector<Element> & iter)
{
  auto f = [](
    const std::reference_wrapper<const Element> & ref_p1,
    const std::reference_wrapper<const Element> & ref_p2) {
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
  auto rings = MakeReferenceVectorsPerRing(iterator);
  SortEachRingByAngle<ElementType<Iter>>(rings);
  return rings;
}

template<typename PointT>
std::vector<std::pair<CloudConstIterator<PointT>, CloudConstIterator<PointT>>>
ExtractSectionsByRing(const typename pcl::PointCloud<PointT>::Ptr & cloud)
{
  const auto & points = cloud->points;

  if (points.size() == 0) {
    return {};
  }

  using T = CloudConstIterator<PointT>;

  const T cloud_begin = points.begin();

  std::set<std::uint16_t> rings;
  std::vector<std::pair<T, T>> sections;

  T begin = cloud_begin;
  std::uint16_t prev_ring = points.at(0).ring;

  for (unsigned int i = 1; i < points.size(); i++) {
    const T p = cloud_begin + i;
    if (p->ring == prev_ring) {
      continue;
    }

    if (rings.find(p->ring) != rings.end()) {
      auto s = fmt::format("Ring {} has already appeared", p->ring);
      throw std::invalid_argument(s);
    }

    rings.insert(p->ring);

    sections.push_back(std::make_pair(begin, p));

    begin = p;
    prev_ring = p->ring;
  }

  sections.push_back(std::make_pair(begin, points.end()));

  return sections;
}

#endif  /* _RING_LIDAR_ODOMETRY_H_ */
