// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _CURVATURE_LIDAR_ODOMETRY_H_
#define _CURVATURE_LIDAR_ODOMETRY_H_

#include "math.hpp"
#include "convolution.hpp"

std::vector<double> CalcCurvature(const std::vector<double> & range)
{
  const std::vector<double> weight{1., 1., 1., 1., 1., -10., 1., 1., 1., 1., 1.};
  auto f = [](const double v) {return v * v;};
  const auto weighted = Convolution1D(range.begin(), range.end(), weight.begin(), weight.end());
  return weighted | ranges::views::transform(f) | ranges::to_vector;
}

#endif  /* _CURVATURE_LIDAR_ODOMETRY_H_ */
