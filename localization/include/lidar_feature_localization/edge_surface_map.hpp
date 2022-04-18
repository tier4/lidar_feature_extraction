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

#ifndef LIDAR_FEATURE_LOCALIZATION__EDGE_SURFACE_MAP_HPP_
#define LIDAR_FEATURE_LOCALIZATION__EDGE_SURFACE_MAP_HPP_

#include <string>

#include "lidar_feature_localization/edge_surface_tuple.hpp"
#include "lidar_feature_localization/map_io.hpp"
#include "lidar_feature_localization/recent_scans.hpp"

class EdgeSurfaceMap
{
public:
  explicit EdgeSurfaceMap(const int n_local_scans)
  : n_local_scans_(n_local_scans) {}

  bool IsEmpty() const
  {
    return edge_scans_.IsEmpty() && surface_scans_.IsEmpty();
  }

  void Add(const Eigen::Isometry3d & pose, const EdgeSurfaceTuple & edge_surface_scan)
  {
    const auto edge_scan = std::get<0>(edge_surface_scan);
    const auto surface_scan = std::get<1>(edge_surface_scan);

    edge_scans_.Add(pose, edge_scan);
    surface_scans_.Add(pose, surface_scan);
  }

  EdgeSurfaceTuple GetRecent() const
  {
    return std::make_tuple(
      edge_scans_.GetRecent(n_local_scans_),
      surface_scans_.GetRecent(n_local_scans_)
    );
  }

  void Save(const std::string & dirname) const
  {
    SaveMapIfNotEmpty<pcl::PointXYZ>(dirname + "/" + "edge.pcd", edge_scans_.GetAll());
    SaveMapIfNotEmpty<pcl::PointXYZ>(dirname + "/" + "surface.pcd", surface_scans_.GetAll());
  }

private:
  const int n_local_scans_;
  RecentScans edge_scans_;
  RecentScans surface_scans_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__EDGE_SURFACE_MAP_HPP_
