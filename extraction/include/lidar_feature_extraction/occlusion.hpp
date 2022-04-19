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

#ifndef LIDAR_FEATURE_EXTRACTION__OCCLUSION_HPP_
#define LIDAR_FEATURE_EXTRACTION__OCCLUSION_HPP_

#include <vector>

#include "lidar_feature_extraction/label.hpp"
#include "lidar_feature_extraction/neighbor.hpp"

template<typename PointT>
void FromLeft(
  std::vector<PointLabel> & labels,
  const NeighborCheckXY<PointT> & is_neighbor,
  const Range<PointT> & range,
  const unsigned int padding,
  const double distance_diff_threshold)
{
  for (unsigned int i = 0; i < labels.size() - padding - 1; i++) {
    if (!is_neighbor(i + 0, i + 1)) {
      continue;
    }

    const double range0 = range(i + 0);
    const double range1 = range(i + 1);

    if (range1 > range0 + distance_diff_threshold) {
      FillFromLeft(labels, is_neighbor, i + 1, i + padding + 2, PointLabel::Occluded);
    }
  }
}

template<typename PointT>
void FromRight(
  std::vector<PointLabel> & labels,
  const NeighborCheckXY<PointT> & is_neighbor,
  const Range<PointT> & range,
  const unsigned int padding,
  const double distance_diff_threshold)
{
  for (unsigned int i = labels.size() - 1; i >= padding + 1; i--) {
    if (!is_neighbor(i - 0, i - 1)) {
      continue;
    }

    const double range1 = range(i - 1);
    const double range0 = range(i - 0);

    if (range1 > range0 + distance_diff_threshold) {
      FillFromRight(labels, is_neighbor, i - padding - 2, i - 1, PointLabel::Occluded);
    }
  }
}

template<typename PointT>
void LabelOccludedPoints(
  std::vector<PointLabel> & labels,
  const NeighborCheckXY<PointT> & is_neighbor,
  const Range<PointT> & range,
  const unsigned int padding,
  const double distance_diff_threshold)
{
  FromLeft(labels, is_neighbor, range, padding, distance_diff_threshold);
  FromRight(labels, is_neighbor, range, padding, distance_diff_threshold);
}

#endif  // LIDAR_FEATURE_EXTRACTION__OCCLUSION_HPP_
