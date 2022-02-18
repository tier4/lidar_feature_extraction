// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _CURVATURE_LIDAR_ODOMETRY_H_
#define _CURVATURE_LIDAR_ODOMETRY_H_

#include <algorithm>
#include <vector>

#include "math.hpp"
#include "convolution.hpp"

std::vector<double> MakeWeight(const int padding)
{
  assert(padding > 0);
  std::vector<double> weight(padding * 2 + 1, 1.);
  weight.at(padding) = -2. * padding;
  return weight;
}

std::vector<double> CalcCurvature(const std::vector<double> & range, int padding)
{
  const std::vector<double> weight = MakeWeight(padding);
  auto f = [](const double v) {return v * v;};
  const auto weighted = Convolution1D(range.begin(), range.end(), weight.begin(), weight.end());
  return weighted | ranges::views::transform(f) | ranges::to_vector;
}

class Curvature
{
public:
  Curvature(
    const std::vector<double> & curvature,
    const double edge_threshold,
    const double surface_threshold)
  : curvature_(curvature),
    edge_threshold_(edge_threshold),
    surface_threshold_(surface_threshold)
  {
  }

  bool IsEdge(const int i) const
  {
    return curvature_.at(i) >= edge_threshold_;
  }

  bool IsSurface(const int i) const
  {
    return curvature_.at(i) <= surface_threshold_;
  }

  int Size() const
  {
    return curvature_.size();
  }

private:
  const std::vector<double> curvature_;
  const double edge_threshold_;
  const double surface_threshold_;
};

#endif  /* _CURVATURE_LIDAR_ODOMETRY_H_ */
