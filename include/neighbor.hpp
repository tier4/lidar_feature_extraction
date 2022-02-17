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

#endif /* _NEIGHBOR_LIDAR_ODOMETRY_H_ */
