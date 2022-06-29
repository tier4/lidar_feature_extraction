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

#include <gmock/gmock.h>

#include <Eigen/Geometry>

#include <vector>
#include <memory>

#include "lidar_feature_localization/odometry.hpp"
#include "lidar_feature_localization/pose_updater.hpp"
#include "lidar_feature_localization/point_cloud_map.hpp"


class PoseUpdater
{
public:
  explicit PoseUpdater(const std::vector<Eigen::Vector3d> &)
  {
  }

  Eigen::Isometry3d operator()(
    const Eigen::Vector3d &,
    const Eigen::Isometry3d & pose) const
  {
    Eigen::Isometry3d dpose;
    dpose.linear() = Eigen::Matrix3d::Identity();
    dpose.translation() = Eigen::Vector3d(1., 2., 0.);
    return dpose * pose;
  }
};

class Map
{
public:
  Map()
  {
  }

  std::vector<Eigen::Vector3d> GetRecent() const
  {
    return map_;
  }

  int size() const
  {
    return map_.size();
  }

  bool IsEmpty() const
  {
    return map_.size() == 0;
  }

  void Add(const Eigen::Isometry3d & pose, const Eigen::Vector3d & point)
  {
    map_.push_back(pose * point);
  }

private:
  std::vector<Eigen::Vector3d> map_;
};

TEST(Odometry, Update)
{
  auto map = std::make_shared<Map>();

  EXPECT_TRUE(map->IsEmpty());

  Odometry<PoseUpdater, Map, Eigen::Vector3d> odometry(map);
  EXPECT_EQ(map->size(), 0);

  odometry.Update(Eigen::Vector3d(0., 0., 0.));
  EXPECT_EQ((odometry.CurrentPose().translation() - Eigen::Vector3d(0., 0., 0.)).norm(), 0.);
  EXPECT_EQ(map->size(), 1);

  odometry.Update(Eigen::Vector3d(0., 0., 0.));
  EXPECT_EQ((odometry.CurrentPose().translation() - Eigen::Vector3d(1., 2., 0.)).norm(), 0.);
  EXPECT_EQ(map->size(), 2);

  odometry.Update(Eigen::Vector3d(0., 0., 0.));
  EXPECT_EQ((odometry.CurrentPose().translation() - Eigen::Vector3d(2., 4., 0.)).norm(), 0.);
  EXPECT_EQ(map->size(), 3);

  const std::vector<Eigen::Vector3d> points = map->GetRecent();
  EXPECT_EQ((points.at(0) - Eigen::Vector3d(0., 0., 0.)).norm(), 0.);
  EXPECT_EQ((points.at(1) - Eigen::Vector3d(1., 2., 0.)).norm(), 0.);
  EXPECT_EQ((points.at(2) - Eigen::Vector3d(2., 4., 0.)).norm(), 0.);
}
