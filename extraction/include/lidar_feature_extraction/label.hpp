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


#ifndef LIDAR_FEATURE_EXTRACTION__LABEL_HPP_
#define LIDAR_FEATURE_EXTRACTION__LABEL_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <string>
#include <vector>

#include <boost/range/adaptor/reversed.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lidar_feature_extraction/algorithm.hpp"
#include "lidar_feature_extraction/cloud_iterator.hpp"
#include "lidar_feature_extraction/fill.hpp"
#include "lidar_feature_extraction/index_range.hpp"
#include "lidar_feature_extraction/mapped_points.hpp"
#include "lidar_feature_extraction/neighbor.hpp"
#include "lidar_feature_extraction/range.hpp"
#include "lidar_feature_extraction/range_message.hpp"
#include "lidar_feature_extraction/point_label.hpp"

#include "lidar_feature_library/point_type.hpp"


inline std::vector<PointLabel> InitLabels(const int size)
{
  return std::vector<PointLabel>(size, PointLabel::Default);
}

class EdgeLabel
{
public:
  EdgeLabel(
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
    assert(curvature.size() == labels.size());
    assert(is_neighbor.size() == static_cast<int>(labels.size()));

    auto is_edge = [&](const int i) {
        return curvature.at(i) >= threshold_;
      };

    const std::vector<int> indices = Argsort(curvature.begin(), curvature.end());

    for (const int index : boost::adaptors::reverse(indices)) {
      if (!(labels.at(index) == PointLabel::Default && is_edge(index))) {
        continue;
      }

      FillNeighbors(labels, is_neighbor, index, padding_, PointLabel::EdgeNeighbor);
      labels.at(index) = PointLabel::Edge;
    }
  }

private:
  const int padding_;
  const double threshold_;
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
    assert(is_neighbor.size());
    auto is_surface = [&](const int i) {
        return curvature.at(i) <= threshold_;
      };

    const std::vector<int> indices = Argsort(curvature.begin(), curvature.end());

    for (const int index : indices) {
      if (!(labels.at(index) == PointLabel::Default && is_surface(index))) {
        continue;
      }

      FillNeighbors(labels, is_neighbor, index, padding_, PointLabel::SurfaceNeighbor);
      labels.at(index) = PointLabel::Surface;
    }
  }

private:
  const int padding_;
  const double threshold_;
};

template<typename PointT>
void AssignLabel(
  std::vector<PointLabel> & labels,
  const std::vector<double> & curvature,
  const NeighborCheckXY<PointT> & is_neighbor,
  const PaddedIndexRange & index_range,
  const EdgeLabel & edge_label,
  const SurfaceLabel & surface_label)
{
  assert(curvature.size() == labels.size());
  assert(is_neighbor.size() == static_cast<int>(labels.size()));

  for (int j = 0; j < index_range.NBlocks(); j++) {
    const int begin = index_range.Begin(j);
    const int end = index_range.End(j);

    span<PointLabel> label_view(labels.begin() + begin, labels.begin() + end);
    const const_span<double> curvature_view(curvature.begin() + begin, curvature.begin() + end);
    const NeighborCheckXY<PointT> sliced_neighbor = is_neighbor.Slice(begin, end);

    edge_label.Assign(label_view, curvature_view, sliced_neighbor);
    surface_label.Assign(label_view, curvature_view, sliced_neighbor);
  }
}

template<typename InputPointT>
void AppendXYZIR(
  typename pcl::PointCloud<PointXYZIR>::Ptr output_cloud,
  const std::vector<InputPointT> & points,
  const std::vector<double> & curvature)
{
  assert(points.size() == curvature.size());

  for (size_t i = 0; i < points.size(); i++) {
    const InputPointT & p = points[i];
    const PointXYZIR q(p.x, p.y, p.z, curvature[i], p.ring);
    output_cloud->push_back(q);
  }
}

#endif  // LIDAR_FEATURE_EXTRACTION__LABEL_HPP_
