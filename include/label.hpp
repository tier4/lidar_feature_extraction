// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _LABEL_LIDAR_ODOMETRY_H_
#define _LABEL_LIDAR_ODOMETRY_H_

#include <vector>

#include "algorithm.hpp"
#include "curvature_label.hpp"
#include "index_range.hpp"

template<typename PointT>
class Label
{
public:
  Label(
    const std::vector<double> & curvature,
    const int padding,
    const int offset,
    const double edge_threshold,
    const double surface_threshold)
  : curvature_(Curvature(curvature, edge_threshold, surface_threshold)),
    indices_(Argsort(curvature)),
    padding_(padding),
    offset_(offset)
  {
  }

  void Edge(
    std::vector<CurvatureLabel> & labels, Mask<PointT> & mask,
    const int n_max_edges) const
  {
    int n_picked = 0;
    for (const int index : boost::adaptors::reverse(indices_)) {
      if (n_picked >= n_max_edges) {
        break;
      }
      if (mask.At(offset_ + index) || !curvature_.IsEdge(index)) {
        continue;
      }

      labels.at(offset_ + index) = CurvatureLabel::Edge;

      mask.FillNeighbors(offset_ + index, padding_);

      n_picked++;
    }
  }

  void Surface(std::vector<CurvatureLabel> & labels, Mask<PointT> & mask) const
  {
    for (const int index : indices_) {
      if (mask.At(offset_ + index) || !curvature_.IsSurface(index)) {
        continue;
      }

      labels.at(offset_ + index) = CurvatureLabel::Surface;

      mask.FillNeighbors(offset_ + index, padding_);
    }
  }

private:
  const Curvature curvature_;
  const std::vector<int> indices_;
  const int padding_;
  const int offset_;
};

template<typename PointT>
std::vector<CurvatureLabel> AssignLabel(
  const Mask<PointT> & input_mask,
  const Range<PointT> & range,
  const int n_blocks,
  const int padding,
  const int max_edges_per_block,
  const double edge_threshold,
  const double surface_threshold)
{
  Mask mask = input_mask;  // copy to make argument const

  std::vector<CurvatureLabel> labels(mask.Size(), CurvatureLabel::Default);
  const PaddedIndexRange index_range(0, mask.Size(), n_blocks, padding);
  for (int j = 0; j < n_blocks; j++) {
    const std::vector<double> ranges = range(index_range.Begin(j), index_range.End(j));
    const std::vector<double> curvature = CalcCurvature(ranges, padding);

    const int expected_size = index_range.End(j) - index_range.Begin(j) - 2 * padding;
    assert(curvature.size() == static_cast<std::uint32_t>(expected_size));

    const int offset = index_range.Begin(j) + padding;

    const Label<PointT> label(curvature, padding, offset, edge_threshold, surface_threshold);
    label.Edge(labels, mask, max_edges_per_block);
    label.Surface(labels, mask);
  }

  return labels;
}

#endif  /* _LABEL_LIDAR_ODOMETRY_H_ */
