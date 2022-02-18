// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _LABEL_EDGES_LIDAR_ODOMETRY_H_
#define _LABEL_EDGES_LIDAR_ODOMETRY_H_

#include <vector>

template<typename PointT>
void LabelEdges(
  std::vector<CurvatureLabel>::iterator & label_begin,
  Mask<PointT> & mask,
  const std::vector<double>::const_iterator & curvature_begin,
  const std::vector<int> & indices,
  const int n_max_edges,
  const double edge_threshold)
{
  int n_picked = 0;
  for (const int index : boost::adaptors::reverse(indices)) {
    if (mask.At(offset + index) || *(curvature_begin + index) <= edge_threshold) {
      continue;
    }

    if (n_picked >= n_max_edges) {
      break;
    }

    n_picked++;

    *(label_begin + index) = CurvatureLabel::Edge;

    mask.FillNeighbors(offset + index, padding);
  }
}

#ifndef  /* _LABEL_EDGES_LIDAR_ODOMETRY_H_ */
