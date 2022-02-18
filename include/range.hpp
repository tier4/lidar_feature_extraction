// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef RANGE_HPP_
#define RANGE_HPP_

#include <vector>

#include "cloud_iterator.hpp"
#include "reference_wrapper.hpp"

template<typename PointT>
class Range
{
public:
  explicit Range(const ConstReferenceVector<PointT> & ref_points)
  : ref_points_(ref_points) {}

  double operator()(const int index) const
  {
    const PointT & p = ref_points_.at(index).get();
    return XYNorm(p.x, p.y);
  }

  std::vector<double> operator()(const int begin, const int end) const
  {
    std::vector<double> ranges(end - begin);
    for (int i = begin; i < end; i++) {
      ranges[i] = (*this)(i);
    }
    return ranges;
  }

private:
  const ConstReferenceVector<PointT> ref_points_;
};

#endif  // RANGE_HPP_
