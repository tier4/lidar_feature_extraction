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

class Label
{
public:
  explicit Label(const std::vector<double> & curvature)
  : curvature_(curvature),
    indices_(Argsort(curvature))
  {
  }

protected:
  const std::vector<double> curvature_;
  const std::vector<int> indices_;
};

template<typename PointT>
class EdgeLabel : public Label
{
public:
  EdgeLabel(
    const std::vector<double> & curvature,
    const int padding,
    const int offset,
    const double threshold,
    const int n_max_edges)
  : Label(curvature),
    padding_(padding),
    offset_(offset),
    threshold_(threshold),
    n_max_edges_(n_max_edges)
  {
  }

  void Assign(std::vector<CurvatureLabel> & labels, Mask<PointT> & mask) const
  {
    int n_picked = 0;
    for (const int index : boost::adaptors::reverse(indices_)) {
      if (n_picked >= n_max_edges_) {
        break;
      }
      if (mask.At(offset_ + index) || !this->IsEdge(index)) {
        continue;
      }

      labels.at(offset_ + index) = CurvatureLabel::Edge;

      mask.FillNeighbors(offset_ + index, padding_);

      n_picked++;
    }
  }

private:
  bool IsEdge(const int i) const
  {
    return curvature_.at(i) >= threshold_;
  }

  const int padding_;
  const int offset_;
  const double threshold_;
  const int n_max_edges_;
};

template<typename PointT>
class SurfaceLabel : public Label
{
public:
  SurfaceLabel(
    const std::vector<double> & curvature,
    const int padding,
    const int offset,
    const double threshold)
  : Label(curvature),
    padding_(padding),
    offset_(offset),
    threshold_(threshold)
  {
  }

  void Assign(std::vector<CurvatureLabel> & labels, Mask<PointT> & mask) const
  {
    for (const int index : indices_) {
      if (mask.At(offset_ + index) || !this->IsSurface(index)) {
        continue;
      }

      labels.at(offset_ + index) = CurvatureLabel::Surface;

      mask.FillNeighbors(offset_ + index, padding_);
    }
  }

private:
  bool IsSurface(const int i) const
  {
    return curvature_.at(i) <= threshold_;
  }

  const int padding_;
  const int offset_;
  const double threshold_;
};

template<typename PointT>
std::vector<CurvatureLabel> AssignLabel(
  const Mask<PointT> & input_mask,
  const Range<PointT> & range,
  const int n_blocks,
  const int padding,
  const int n_max_edges,
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

    const EdgeLabel<PointT> edge_label(curvature, padding, offset, edge_threshold, n_max_edges);
    const SurfaceLabel<PointT> surface_label(curvature, padding, offset, surface_threshold);
    edge_label.Assign(labels, mask);
    surface_label.Assign(labels, mask);
  }

  return labels;
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
