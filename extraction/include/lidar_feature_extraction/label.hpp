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

#include "lidar_feature_extraction/algorithm.hpp"
#include "lidar_feature_extraction/curvature.hpp"
#include "lidar_feature_extraction/index_range.hpp"
#include "lidar_feature_extraction/mapped_points.hpp"
#include "lidar_feature_extraction/mask.hpp"
#include "lidar_feature_extraction/point_label.hpp"

#include "lidar_feature_library/point_type.hpp"

template<typename PointT>
class EdgeLabel
{
public:
  EdgeLabel(
    const int padding,
    const double threshold,
    const int n_max_edges)
  : padding_(padding),
    threshold_(threshold),
    n_max_edges_(n_max_edges)
  {
  }

  void Assign(
    std::vector<PointLabel> & labels,
    Mask<PointT> & mask,
    const std::vector<double> & curvature,
    const int offset) const
  {
    auto is_edge = [&](const int i) {
        return curvature.at(i) >= threshold_;
      };

    const std::vector<int> indices = Argsort(curvature);

    int n_picked = 0;
    for (const int index : boost::adaptors::reverse(indices)) {
      if (n_picked >= n_max_edges_) {
        break;
      }
      if (mask.At(offset + index) || !is_edge(index)) {
        continue;
      }

      labels.at(offset + index) = PointLabel::Edge;

      mask.FillNeighbors(offset + index, padding_);

      n_picked++;
    }
  }

private:
  const int padding_;
  const double threshold_;
  const int n_max_edges_;
};

template<typename PointT>
class SurfaceLabel
{
public:
  SurfaceLabel(
    const int padding,
    const double threshold)
  : padding_(padding),
    threshold_(threshold)
  {
  }

  void Assign(
    std::vector<PointLabel> & labels,
    Mask<PointT> & mask,
    const std::vector<double> & curvature,
    const int offset) const
  {
    auto is_surface = [&](const int i) {
        return curvature.at(i) <= threshold_;
      };

    const std::vector<int> indices = Argsort(curvature);

    for (const int index : indices) {
      if (mask.At(offset + index) || !is_surface(index)) {
        continue;
      }

      labels.at(offset + index) = PointLabel::Surface;

      mask.FillNeighbors(offset + index, padding_);
    }
  }

private:
  const int padding_;
  const double threshold_;
};

template<typename PointT>
std::vector<PointLabel> AssignLabel(
  const Mask<PointT> & input_mask,
  const Range<PointT> & range,
  const EdgeLabel<PointT> edge_label,
  const SurfaceLabel<PointT> surface_label,
  const int n_blocks,
  const int padding)
{
  Mask mask = input_mask;  // copy to make argument const

  const PaddedIndexRange index_range(0, mask.Size(), n_blocks, padding);

  std::vector<PointLabel> labels(mask.Size(), PointLabel::Default);

  for (int j = 0; j < n_blocks; j++) {
    const std::vector<double> ranges = range(index_range.Begin(j), index_range.End(j));
    const std::vector<double> curvature = CalcCurvature(ranges, padding);

    const int expected_size = index_range.End(j) - index_range.Begin(j) - 2 * padding;
    assert(curvature.size() == static_cast<std::uint32_t>(expected_size));

    const int offset = index_range.Begin(j) + padding;

    edge_label.Assign(labels, mask, curvature, offset);
    surface_label.Assign(labels, mask, curvature, offset);
  }

  return labels;
}

template<typename PointT>
void ExtractByLabel(
  typename pcl::PointCloud<PointT>::Ptr output_cloud,
  const MappedPoints<PointT> & ref_points,
  const std::vector<PointLabel> & labels,
  const PointLabel & label)
{
  assert(ref_points.size() == static_cast<int>(labels.size()));

  for (unsigned int i = 0; i < labels.size(); i++) {
    if (labels.at(i) == label) {
      output_cloud->push_back(ref_points.at(i));
    }
  }
}

#endif  // LABEL_HPP_
