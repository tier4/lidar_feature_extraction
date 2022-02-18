// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _NEIGHBOR_LIDAR_ODOMETRY_H_
#define _NEIGHBOR_LIDAR_ODOMETRY_H_

#include "math.hpp"

template<typename PointT>
bool IsNeighbor(const PointT & p1, const PointT & p2, const double radian_threshold)
{
  return CalcRadian(p1->x, p1->y, p2->x, p2->y) <= radian_threshold;
}

template<typename PointT>
class Neighbor
{
public:
  Neighbor(const CloudConstIterator<PointT> & cloud_begin, const double radian_threshold)
  : cloud_begin_(cloud_begin), radian_threshold_(radian_threshold)
  {
  }

  bool operator()(const int index1, const int index2) const
  {
    const auto & p1 = cloud_begin_ + index1;
    const auto & p2 = cloud_begin_ + index2;
    return IsNeighbor(p1, p2, radian_threshold_);
  }

private:
  const CloudConstIterator<PointT> cloud_begin_;
  const double radian_threshold_;
};

#endif /* _NEIGHBOR_LIDAR_ODOMETRY_H_ */
