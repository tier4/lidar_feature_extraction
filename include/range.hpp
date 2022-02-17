// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _RANGE_LIDAR_ODOMETRY_H_
#define _RANGE_LIDAR_ODOMETRY_H_

#include "cloud_iterator.hpp"

template<typename PointT>
class Range
{
public:
  Range(const CloudConstIterator<PointT> & cloud_begin) : cloud_begin_(cloud_begin) {}

  double operator()(const int index) const
  {
    const auto p = cloud_begin_ + index;
    return XYNorm(p->x, p->y);
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
  const CloudConstIterator<PointT> cloud_begin_;
};

#endif /* _RANGE_LIDAR_ODOMETRY_H_ */
