// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _NEIGHBOR_LIDAR_ODOMETRY_H_
#define _NEIGHBOR_LIDAR_ODOMETRY_H_

#include "math.hpp"
#include "reference_wrapper.hpp"

template<typename PointT>
bool IsNeighbor(const PointT & p1, const PointT & p2, const double radian_threshold)
{
  return CalcRadian(p1.x, p1.y, p2.x, p2.y) <= radian_threshold;
}

template<typename PointT>
class Neighbor
{
public:
  Neighbor(const ConstReferenceVector<PointT> & ref_points, const double radian_threshold)
  : ref_points_(ref_points), radian_threshold_(radian_threshold)
  {
  }

  bool operator()(const int index1, const int index2) const
  {
    const PointT & p1 = ref_points_.at(index1).get();
    const PointT & p2 = ref_points_.at(index2).get();
    return IsNeighbor(p1, p2, radian_threshold_);
  }

private:
  const ConstReferenceVector<PointT> ref_points_;
  const double radian_threshold_;
};

#endif /* _NEIGHBOR_LIDAR_ODOMETRY_H_ */
