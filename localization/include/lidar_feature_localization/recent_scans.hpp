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

#ifndef LIDAR_FEATURE_LOCALIZATION__RECENT_SCANS_HPP_
#define LIDAR_FEATURE_LOCALIZATION__RECENT_SCANS_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>

#include <algorithm>
#include <vector>

#include "lidar_feature_library/pcl_utils.hpp"


template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr MergeClouds(
  const std::vector<typename pcl::PointCloud<PointType>::Ptr> & scans)
{
  typename pcl::PointCloud<PointType>::Ptr merged(new pcl::PointCloud<PointType>());
  for (const auto & scan : scans) {
    *merged += *scan;
  }
  return merged;
}


template<typename PointType>
class RecentScans
{
public:
  RecentScans() {}

  void Add(
    const Eigen::Isometry3d & point_to_map,
    const typename pcl::PointCloud<PointType>::Ptr & scan)
  {
    const auto transformed = TransformPointCloud<PointType>(point_to_map, scan);
    scans_.push_back(transformed);
  }

  bool IsEmpty() const
  {
    return scans_.size() == 0;
  }

  typename pcl::PointCloud<PointType>::Ptr GetRecent(const int n) const
  {
    const int n_recent = std::min(n, static_cast<int>(scans_.size()));
    std::vector recent(scans_.end() - n_recent, scans_.end());
    return MergeClouds<PointType>(recent);
  }

  typename pcl::PointCloud<PointType>::Ptr GetAll() const
  {
    return MergeClouds<PointType>(scans_);
  }

private:
  std::vector<typename pcl::PointCloud<PointType>::Ptr> scans_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__RECENT_SCANS_HPP_
