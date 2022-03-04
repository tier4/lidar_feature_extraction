// Copyright 2022 Tixiao Shan, Takeshi Ishita
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Tixiao Shan, Takeshi Ishita nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


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

#include "lidar_feature_library/point_type.hpp"

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

  const PaddedIndexRange index_range(0, mask.Size(), n_blocks, padding);

  std::vector<CurvatureLabel> labels(mask.Size(), CurvatureLabel::Default);

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

template<typename PointT>
void ExtractByLabel(
  typename pcl::PointCloud<PointT>::Ptr output_cloud,
  const MappedPoints<PointT> & ref_points,
  const std::vector<CurvatureLabel> & labels,
  const CurvatureLabel & label)
{
  assert(ref_points.size() == static_cast<int>(labels.size()));

  for (unsigned int i = 0; i < labels.size(); i++) {
    if (labels.at(i) == label) {
      output_cloud->push_back(ref_points.at(i));
    }
  }
}

#endif  // LABEL_HPP_
