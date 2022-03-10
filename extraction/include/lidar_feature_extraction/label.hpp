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

#include <string>
#include <vector>

#include "lidar_feature_extraction/algorithm.hpp"
#include "lidar_feature_extraction/cloud_iterator.hpp"
#include "lidar_feature_extraction/curvature.hpp"
#include "lidar_feature_extraction/index_range.hpp"
#include "lidar_feature_extraction/label.hpp"
#include "lidar_feature_extraction/mapped_points.hpp"
#include "lidar_feature_extraction/neighbor.hpp"
#include "lidar_feature_extraction/range.hpp"
#include "lidar_feature_extraction/range_message.hpp"
#include "lidar_feature_extraction/point_label.hpp"

#include "lidar_feature_library/point_type.hpp"

template<typename Element>
class Label
{
public:
  Label(
    const MappedPoints<Element> & ref_points,
    const double radian_threshold)
  : label_(std::vector<bool>(ref_points.size(), false)),
    ref_points_(ref_points),
    radian_threshold_(radian_threshold)
  {
  }

  Label(const Label & label)
  : label_(label.label_),
    ref_points_(label.ref_points_),
    radian_threshold_(label.radian_threshold_)
  {
  }

  void Fill(const int index)
  {
    label_.at(index) = true;
  }

  void FillFromLeft(const int begin_index, const int end_index)
  {
    if (end_index > this->Size()) {
      auto s = RangeMessageLargerThan(
        "end_index", "this->Size()", end_index, this->Size());
      throw std::invalid_argument(s);
    }

    if (begin_index < 0) {
      auto s = RangeMessageSmallerThan("begin_index", "0", begin_index, 0);
      throw std::invalid_argument(s);
    }

    for (int i = begin_index; i < end_index - 1; i++) {
      label_.at(i) = true;

      const Element & p0 = ref_points_.at(i + 0);
      const Element & p1 = ref_points_.at(i + 1);
      if (!IsNeighbor(p0, p1, radian_threshold_)) {
        return;
      }
    }
    label_.at(end_index - 1) = true;
  }

  void FillFromRight(const int begin_index, const int end_index)
  {
    if (end_index >= this->Size()) {
      auto s = RangeMessageLargerThanOrEqualTo(
        "end_index", "this->Size()", end_index, this->Size());
      throw std::invalid_argument(s);
    }

    if (begin_index < -1) {
      auto s = RangeMessageSmallerThan("begin_index", "-1", begin_index, -1);
      throw std::invalid_argument(s);
    }

    for (int i = end_index; i > begin_index + 1; i--) {
      label_.at(i) = true;

      const Element & p0 = ref_points_.at(i - 0);
      const Element & p1 = ref_points_.at(i - 1);
      if (!IsNeighbor(p0, p1, radian_threshold_)) {
        return;
      }
    }
    label_.at(begin_index + 1) = true;
  }

  void FillNeighbors(const int index, const int padding)
  {
    if (index + padding >= this->Size()) {
      auto s = RangeMessageLargerThanOrEqualTo(
        "index + padding", "this->Size()", index + padding, this->Size());
      throw std::invalid_argument(s);
    }

    if (index - padding < 0) {
      auto s = RangeMessageSmallerThan(
        "index - padding", "0", index - padding, 0);
      throw std::invalid_argument(s);
    }

    this->Fill(index);
    this->FillFromLeft(index + 1, index + 1 + padding);
    this->FillFromRight(index - padding - 1, index - 1);
  }

  bool At(const int index) const
  {
    return label_.at(index);
  }

  std::vector<bool> Get() const
  {
    return label_;
  }

  int Size() const
  {
    return label_.size();
  }

private:
  std::vector<bool> label_;
  const MappedPoints<Element> ref_points_;
  const double radian_threshold_;
};

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
    Label<PointT> & label,
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
      if (label.At(offset + index) || !is_edge(index)) {
        continue;
      }

      labels.at(offset + index) = PointLabel::Edge;

      label.FillNeighbors(offset + index, padding_);

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
    Label<PointT> & label,
    const std::vector<double> & curvature,
    const int offset) const
  {
    auto is_surface = [&](const int i) {
        return curvature.at(i) <= threshold_;
      };

    const std::vector<int> indices = Argsort(curvature);

    for (const int index : indices) {
      if (label.At(offset + index) || !is_surface(index)) {
        continue;
      }

      labels.at(offset + index) = PointLabel::Surface;

      label.FillNeighbors(offset + index, padding_);
    }
  }

private:
  const int padding_;
  const double threshold_;
};

template<typename PointT>
void LabelOutOfRange(
  Label<PointT> & label,
  const Range<PointT> & range,
  const double min_range,
  const double max_range)
{
  for (int i = 0; i < range.Size(); i++) {
    if (!IsInInclusiveRange(range(i), min_range, max_range)) {
      label.Fill(i);
    }
  }
}

template<typename PointT>
void LabelOccludedPoints(
  Label<PointT> & label,
  const Neighbor<PointT> & is_neighbor,
  const Range<PointT> & range,
  const int padding,
  const double distance_diff_threshold)
{
  for (int i = padding; i < label.Size() - padding - 1; i++) {
    if (!is_neighbor(i + 0, i + 1)) {
      continue;
    }

    const double range0 = range(i + 0);
    const double range1 = range(i + 1);

    if (range0 > range1 + distance_diff_threshold) {
      label.FillFromRight(i - padding - 1, i);
    }

    if (range1 > range0 + distance_diff_threshold) {
      label.FillFromLeft(i + 1, i + padding + 2);
    }
  }
}

template<typename PointT>
void LabelParallelBeamPoints(
  Label<PointT> & label,
  const Range<PointT> & range,
  const double range_ratio_threshold)
{
  const std::vector<double> ranges = range(0, label.Size());
  for (int i = 1; i < label.Size() - 1; i++) {
    const float ratio1 = std::abs(ranges.at(i - 1) - ranges.at(i)) / ranges.at(i);
    const float ratio2 = std::abs(ranges.at(i + 1) - ranges.at(i)) / ranges.at(i);

    if (ratio1 > range_ratio_threshold && ratio2 > range_ratio_threshold) {
      label.Fill(i);
    }
  }
}


template<typename PointT>
std::vector<PointLabel> AssignLabel(
  const Label<PointT> & input_label,
  const Range<PointT> & range,
  const EdgeLabel<PointT> edge_label,
  const SurfaceLabel<PointT> surface_label,
  const int n_blocks,
  const int padding)
{
  Label label = input_label;  // copy to make argument const

  const PaddedIndexRange index_range(0, label.Size(), n_blocks, padding);

  std::vector<PointLabel> labels(label.Size(), PointLabel::Default);

  for (int j = 0; j < n_blocks; j++) {
    const std::vector<double> ranges = range(index_range.Begin(j), index_range.End(j));
    const std::vector<double> curvature = CalcCurvature(ranges, padding);

    const int expected_size = index_range.End(j) - index_range.Begin(j) - 2 * padding;
    assert(curvature.size() == static_cast<std::uint32_t>(expected_size));

    const int offset = index_range.Begin(j) + padding;

    edge_label.Assign(labels, label, curvature, offset);
    surface_label.Assign(labels, label, curvature, offset);
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
