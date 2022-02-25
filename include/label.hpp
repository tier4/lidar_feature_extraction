// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef LABEL_HPP_
#define LABEL_HPP_

#include <boost/range/adaptor/reversed.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

#include <vector>

#include "algorithm.hpp"
#include "curvature.hpp"
#include "curvature_label.hpp"
#include "index_range.hpp"
#include "mapped_points.hpp"
#include "mask.hpp"
#include "point_type.hpp"

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
    if (index_range.Begin(j) < 0 || range.Size() < index_range.Begin(j)) {
      RCLCPP_INFO(
        rclcpp::get_logger("lidar_feature_extraction"),
        "mask.Size() = %d, n_blocks = %d, padding = %d",
        mask.Size(), n_blocks, padding);
      RCLCPP_INFO(
        rclcpp::get_logger("lidar_feature_extraction"),
        "index_range.Begin(%d) = %d", j, index_range.Begin(j));
    }

    if (index_range.End(j) < 0 || range.Size() < index_range.End(j)) {
      RCLCPP_INFO(
        rclcpp::get_logger("lidar_feature_extraction"),
        "index_range.End(%d) = %d", j, index_range.End(j));
    }

    const std::vector<double> ranges = range(index_range.Begin(j), index_range.End(j));
    std::vector<double> curvature;

    try {
      curvature = CalcCurvature(ranges, padding);
    } catch (const std::invalid_argument & e) {
      continue;
    }

    const int expected_size = index_range.End(j) - index_range.Begin(j) - 2 * padding;
    assert(curvature.size() == static_cast<std::uint32_t>(expected_size));

    const int offset = index_range.Begin(j) + padding;

    const Label<PointT> label(curvature, padding, offset, edge_threshold, surface_threshold);
    label.Edge(labels, mask, max_edges_per_block);
    label.Surface(labels, mask);
  }

  return labels;
}

template<typename Element>
std::vector<CurvatureLabel> AssignLabels(
  const MappedPoints<Element> & ref_points,
  const int n_blocks)
{
  const int padding = 2;
  const int max_edges_per_block = 20;
  const double radian_threshold = 2.0;
  const double distance_diff_threshold = 0.3;
  const double range_ratio_threshold = 0.02;
  const double edge_threshold = 0.1;
  const double surface_threshold = 0.1;

  const Neighbor<Element> neighbor(ref_points, radian_threshold);

  const Range<Element> range(ref_points);

  Mask<Element> mask(ref_points, radian_threshold);
  MaskOccludedPoints<Element>(mask, neighbor, range, padding, distance_diff_threshold);
  MaskParallelBeamPoints<Element>(mask, range, range_ratio_threshold);

  return AssignLabel(
    mask, range, n_blocks, padding,
    max_edges_per_block, edge_threshold, surface_threshold);
}

#endif  // LABEL_HPP_
