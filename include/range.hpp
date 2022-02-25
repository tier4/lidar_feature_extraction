// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef RANGE_HPP_
#define RANGE_HPP_

#include <vector>

#include "cloud_iterator.hpp"
#include "iterator.hpp"
#include "math.hpp"
#include "mapped_points.hpp"

bool IsInInclusiveRange(const double v, const double min, const double max)
{
  return min <= v && v <= max;
}

template<typename T>
pcl::PointCloud<T> FilterByRange(
  const pcl::PointCloud<T> & points,
  const double range_min,
  const double range_max)
{
  pcl::PointCloud<T> result;
  for (const T & p : points) {
    const double norm = XYNorm(p.x, p.y);
    if (IsInInclusiveRange(norm, range_min, range_max)) {
      result.push_back(p);
    }
  }
  return result;
}

template<typename PointT>
class Range
{
public:
  explicit Range(const MappedPoints<PointT> & ref_points)
  : ref_points_(ref_points) {}

  double operator()(const int index) const
  {
    const PointT & p = ref_points_.at(index);
    return XYNorm(p.x, p.y);
  }

  std::vector<double> operator()(const int begin, const int end) const
  {
    std::vector<double> ranges(end - begin);
    for (unsigned int i = 0; i < ranges.size(); i++) {
      ranges.at(i) = (*this)(i);
    }
    return ranges;
  }

private:
  const MappedPoints<PointT> ref_points_;
};

#endif  // RANGE_HPP_
