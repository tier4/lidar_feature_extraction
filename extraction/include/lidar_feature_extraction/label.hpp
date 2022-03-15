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

#include <algorithm>
#include <string>
#include <vector>

#include "lidar_feature_extraction/algorithm.hpp"
#include "lidar_feature_extraction/cloud_iterator.hpp"
#include "lidar_feature_extraction/curvature.hpp"
#include "lidar_feature_extraction/index_range.hpp"
#include "lidar_feature_extraction/mapped_points.hpp"
#include "lidar_feature_extraction/neighbor.hpp"
#include "lidar_feature_extraction/range.hpp"
#include "lidar_feature_extraction/range_message.hpp"
#include "lidar_feature_extraction/point_label.hpp"

#include "lidar_feature_library/point_type.hpp"


std::vector<PointLabel> InitLabels(const int size)
{
  return std::vector<PointLabel>(size, PointLabel::Default);
}

template<typename Container>
void FillFromLeft(
  Container & labels,
  const NeighborCheckBase & is_neighbor,
  const int begin_index,
  const int end_index,
  const PointLabel & label)
{
  assert(static_cast<int>(labels.size()) == is_neighbor.Size());

  if (end_index > static_cast<int>(labels.size())) {
    auto s = RangeMessageLargerThan("end_index", "labels.size()", end_index, labels.size());
    throw std::invalid_argument(s);
  }

  if (begin_index < 0) {
    auto s = RangeMessageSmallerThan("begin_index", "0", begin_index, 0);
    throw std::invalid_argument(s);
  }

  for (int i = begin_index; i < end_index - 1; i++) {
    labels.at(i) = label;

    if (!is_neighbor(i + 0, i + 1)) {
      return;
    }
  }
  labels.at(end_index - 1) = label;
}

template<typename Container>
void FillFromRight(
  Container & labels,
  const NeighborCheckBase & is_neighbor,
  const int begin_index,
  const int end_index,
  const PointLabel & label)
{
  assert(static_cast<int>(labels.size()) == is_neighbor.Size());

  if (end_index >= static_cast<int>(labels.size())) {
    auto s = RangeMessageLargerThanOrEqualTo(
      "end_index", "labels.size()", end_index, labels.size());
    throw std::invalid_argument(s);
  }

  if (begin_index < -1) {
    auto s = RangeMessageSmallerThan("begin_index", "-1", begin_index, -1);
    throw std::invalid_argument(s);
  }

  for (int i = end_index; i > begin_index + 1; i--) {
    labels.at(i) = label;

    if (!is_neighbor(i - 0, i - 1)) {
      return;
    }
  }
  labels.at(begin_index + 1) = label;
}

template<typename Container>
void FillNeighbors(
  Container & labels,
  const NeighborCheckBase & is_neighbor,
  const int index,
  const int padding,
  const PointLabel & label)
{
  const int label_size = static_cast<int>(labels.size());
  assert(label_size == is_neighbor.Size());

  const int min = std::max(-1, index - padding - 1);
  const int max = std::min(index + 1 + padding, label_size);

  labels.at(index) = label;
  FillFromRight(labels, is_neighbor, min, index - 1, label);
  FillFromLeft(labels, is_neighbor, index + 1, max, label);
}

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

  template<typename ContainerA, typename ContainerB>
  void Assign(
    ContainerA & labels,
    const ContainerB & curvature,
    const NeighborCheckBase & is_neighbor) const
  {
    auto is_edge = [&](const int i) {
        return curvature.at(i) >= threshold_;
      };

    const std::vector<int> indices = Argsort(curvature.begin(), curvature.end());

    int n_picked = 0;
    for (const int index : boost::adaptors::reverse(indices)) {
      if (n_picked >= n_max_edges_) {
        break;
      }
      if (labels.at(offset + index) != PointLabel::Default || !is_edge(index)) {
        continue;
      }

      FillNeighbors(labels, is_neighbor, offset + index, padding_, PointLabel::EdgeNeighbor);
      labels.at(offset + index) = PointLabel::Edge;

      n_picked++;
    }
  }

private:
  const int padding_;
  const double threshold_;
  const int n_max_edges_;
};

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

  template<typename ContainerA, typename ContainerB>
  void Assign(
    ContainerA & labels,
    const ContainerB & curvature,
    const NeighborCheckBase & is_neighbor) const
  {
    assert(is_neighbor.Size());
    auto is_surface = [&](const int i) {
        return curvature.at(i) <= threshold_;
      };

    const std::vector<int> indices = Argsort(curvature.begin(), curvature.end());

    for (const int index : indices) {
      if (labels.at(offset + index) != PointLabel::Default || !is_surface(index)) {
        continue;
      }

      FillNeighbors(labels, is_neighbor, offset + index, padding_, PointLabel::SurfaceNeighbor);
      labels.at(offset + index) = PointLabel::Surface;
    }
  }

private:
  const int padding_;
  const double threshold_;
};

void AssignCurvature(
  std::vector<double> & curvature_output,
  const std::vector<double> & curvature,
  const int begin, const int end)
{
  for (int i = 0; i < end - begin; i++) {
    curvature_output[begin + i] = curvature[i];
  }
}

template<typename PointT>
void AssignLabel(
  std::vector<PointLabel> & labels,
  std::vector<double> & curvature_output,
  const NeighborCheckXY<PointT> & is_neighbor,
  const Range<PointT> & range,
  const EdgeLabel<PointT> edge_label,
  const SurfaceLabel<PointT> surface_label,
  const int n_blocks,
  const int padding)
{
  assert(static_cast<int>(labels.size()) == range.Size());

  curvature_output = std::vector<double>(range.Size(), 0.);

  const PaddedIndexRange index_range(0, range.Size(), n_blocks, padding);

  for (int j = 0; j < n_blocks; j++) {
    const std::vector<double> ranges = range(index_range.Begin(j), index_range.End(j));
    const std::vector<double> curvature = CalcCurvature(ranges, padding);

    const int end = index_range.End(j) - padding;
    const int begin = index_range.Begin(j) + padding;
    assert(curvature.size() == static_cast<std::uint32_t>(end - begin));
    AssignCurvature(curvature_output, curvature, begin, end);

    edge_label.Assign(labels, is_neighbor, curvature, begin);
    surface_label.Assign(labels, is_neighbor, curvature, begin);
  }
}

template<typename PointT>
void ExtractByLabel(
  typename pcl::PointCloud<PointT>::Ptr output_cloud,
  const MappedPoints<PointT> & ref_points,
  const std::vector<PointLabel> & labels,
  const PointLabel & label)
{
  assert(ref_points.Size() == static_cast<int>(labels.size()));

  for (unsigned int i = 0; i < labels.size(); i++) {
    if (labels.at(i) == label) {
      output_cloud->push_back(ref_points.At(i));
    }
  }
}

#endif  // LABEL_HPP_
