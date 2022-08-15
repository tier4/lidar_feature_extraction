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

#ifndef LIDAR_FEATURE_LOCALIZATION__ODOMETRY_HPP_
#define LIDAR_FEATURE_LOCALIZATION__ODOMETRY_HPP_

#include <memory>
#include <string>
#include <tuple>

#include "lidar_feature_library/convert_point_cloud_type.hpp"

#include "lidar_feature_localization/optimizer.hpp"
#include "lidar_feature_localization/recent_scans.hpp"


template<class PoseUpdaterClass, class MapClass, class ScanType>
class Odometry
{
public:
  explicit Odometry(std::shared_ptr<MapClass> & initial_map)
  : pose_(Eigen::Isometry3d::Identity()),
    map_(initial_map)
  {
  }

  void Update(const ScanType & scan)
  {
    if (map_->IsEmpty()) {
      map_->Add(pose_, scan);
      return;
    }

    const PoseUpdaterClass update(map_->GetRecent());
    pose_ = update(scan, pose_);

    map_->Add(pose_, scan);
  }

  Eigen::Isometry3d CurrentPose() const
  {
    return pose_;
  }

private:
  Eigen::Isometry3d pose_;
  std::shared_ptr<MapClass> map_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__ODOMETRY_HPP_
