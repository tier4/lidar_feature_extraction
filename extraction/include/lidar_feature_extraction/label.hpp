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
#include "lidar_feature_extraction/mapped_points.hpp"
#include "lidar_feature_extraction/neighbor.hpp"
#include "lidar_feature_extraction/occlusion.hpp"
#include "lidar_feature_extraction/range.hpp"
#include "lidar_feature_extraction/range_message.hpp"
#include "lidar_feature_extraction/point_label.hpp"

#include "lidar_feature_library/point_type.hpp"

class LabelBase
{
public:
  explicit LabelBase(const int size)
  : label_(std::vector<PointLabel>(size, PointLabel::Default))
  {
  }

  explicit LabelBase(const LabelBase & label)
  : label_(label.label_)
  {
  }

  void Fill(const int index, const PointLabel & label)
  {
    label_.at(index) = label;
  }

  PointLabel At(const int index) const
  {
    return label_.at(index);
  }

  std::vector<PointLabel> Get() const
  {
    return label_;
  }

  int Size() const
  {
    return label_.size();
  }

protected:
  std::vector<PointLabel> label_;
};

template<typename PointT>
class Label : public LabelBase
{
public:
  explicit Label(const NeighborCheck<PointT> & is_neighbor)
  : LabelBase(is_neighbor.Size()), is_neighbor_(is_neighbor)
  {
  }

  explicit Label(const Label & label)
  : LabelBase(label), is_neighbor_(label.is_neighbor_)
  {
  }

  void FillFromLeft(const int begin_index, const int end_index, const PointLabel & label)
  {
    if (end_index > this->Size()) {
      auto s = RangeMessageLargerThan("end_index", "this->Size()", end_index, this->Size());
      throw std::invalid_argument(s);
    }

    if (begin_index < 0) {
      auto s = RangeMessageSmallerThan("begin_index", "0", begin_index, 0);
      throw std::invalid_argument(s);
    }

    for (int i = begin_index; i < end_index - 1; i++) {
      label_.at(i) = label;

      if (!is_neighbor_(i + 0, i + 1)) {
        return;
      }
    }
    label_.at(end_index - 1) = label;
  }

  void FillFromRight(const int begin_index, const int end_index, const PointLabel & label)
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
      label_.at(i) = label;

      if (!is_neighbor_(i - 0, i - 1)) {
        return;
      }
    }
    label_.at(begin_index + 1) = label;
  }

  void FillNeighbors(const int index, const int padding, const PointLabel & label)
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

    this->Fill(index, label);
    this->FillFromLeft(index + 1, index + 1 + padding, label);
    this->FillFromRight(index - padding - 1, index - 1, label);
  }

private:
  const NeighborCheck<PointT> is_neighbor_;
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
    Label<PointT> & label,
    const std::vector<double> & curvature,
    const int offset) const
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
      if (label.At(offset + index) != PointLabel::Default || !is_edge(index)) {
        continue;
      }

      label.FillNeighbors(offset + index, padding_, PointLabel::EdgeNeighbor);
      label.Fill(offset + index, PointLabel::Edge);

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
    Label<PointT> & label,
    const std::vector<double> & curvature,
    const int offset) const
  {
    auto is_surface = [&](const int i) {
        return curvature.at(i) <= threshold_;
      };

    const std::vector<int> indices = Argsort(curvature.begin(), curvature.end());

    for (const int index : indices) {
      if (label.At(offset + index) != PointLabel::Default || !is_surface(index)) {
        continue;
      }

      label.FillNeighbors(offset + index, padding_, PointLabel::SurfaceNeighbor);
      label.Fill(offset + index, PointLabel::Surface);
    }
  }

private:
  const int padding_;
  const double threshold_;
};

template<typename PointT>
void LabelOutOfRange(
  LabelBase & label,
  const Range<PointT> & range,
  const double min_range,
  const double max_range)
{
  for (int i = 0; i < range.Size(); i++) {
    if (!IsInInclusiveRange(range(i), min_range, max_range)) {
      label.Fill(i, PointLabel::OutOfRange);
    }
  }
}

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
  Label<PointT> & label,
  std::vector<double> & curvature_output,
  const Range<PointT> & range,
  const EdgeLabel<PointT> edge_label,
  const SurfaceLabel<PointT> surface_label,
  const int n_blocks,
  const int padding)
{
  assert(label.Size() == range.Size());

  curvature_output = std::vector<double>(range.Size(), 0.);

  const PaddedIndexRange index_range(0, range.Size(), n_blocks, padding);

  for (int j = 0; j < n_blocks; j++) {
    const std::vector<double> ranges = range(index_range.Begin(j), index_range.End(j));
    const std::vector<double> curvature = CalcCurvature(ranges, padding);

    const int end = index_range.End(j) - padding;
    const int begin = index_range.Begin(j) + padding;
    assert(curvature.size() == static_cast<std::uint32_t>(end - begin));
    AssignCurvature(curvature_output, curvature, begin, end);

    edge_label.Assign(label, curvature, begin);
    surface_label.Assign(label, curvature, begin);
  }
}

template<typename PointT>
void ExtractByLabel(
  typename pcl::PointCloud<PointT>::Ptr output_cloud,
  const MappedPoints<PointT> & ref_points,
  const LabelBase & labels,
  const PointLabel & label)
{
  assert(ref_points.Size() == labels.Size());

  for (int i = 0; i < labels.Size(); i++) {
    if (labels.At(i) == label) {
      output_cloud->push_back(ref_points.At(i));
    }
  }
}

#endif  // LABEL_HPP_
