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

#include <Eigen/Core>

#include "lidar_feature_localization/recent_scans.hpp"


TEST(RecentScans, RecentScans)
{
  auto Equal = [](const pcl::PointXYZ & p0, pcl::PointXYZ & p1) {
      return p0.x == p1.x && p0.y == p1.y && p0.z == p1.z;
    };

  RecentScans<pcl::PointXYZ> scans;
  pcl::PointCloud<pcl::PointXYZ>::Ptr expected(new pcl::PointCloud<pcl::PointXYZ>());

  EXPECT_TRUE(scans.IsEmpty());

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
    scan->push_back(pcl::PointXYZ(1, 2, 0));

    Eigen::Isometry3d pose;
    pose.linear() = Eigen::Matrix3d::Identity();
    pose.translation() = Eigen::Vector3d::Zero();
    scans.Add(pose, scan);
  }

  expected->push_back(pcl::PointXYZ(1, 2, 0));

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
    scan->push_back(pcl::PointXYZ(1, 2, 0));

    Eigen::Isometry3d pose;
    pose.linear() = Eigen::Matrix3d::Identity();
    pose.translation() = Eigen::Vector3d(0, 0, 2);
    scans.Add(pose, scan);
  }

  expected->push_back(pcl::PointXYZ(1, 2, 2));

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
    scan->push_back(pcl::PointXYZ(0, 1, 0));

    Eigen::Isometry3d pose;
    pose.linear() = (
      Eigen::Matrix3d() <<
        0, 0, -1,
        -1, 0, 0,
        0, 1, 0
    ).finished();
    pose.translation() = Eigen::Vector3d(0, 2, 0);
    scans.Add(pose, scan);
  }

  expected->push_back(pcl::PointXYZ(0, 2, 1));

  EXPECT_FALSE(scans.IsEmpty());
  {
    const pcl::PointCloud<pcl::PointXYZ> result = *scans.GetAll();

    EXPECT_TRUE(Equal(result.at(0), expected->at(0)));
    EXPECT_TRUE(Equal(result.at(1), expected->at(1)));
    EXPECT_TRUE(Equal(result.at(2), expected->at(2)));
  }

  {
    const pcl::PointCloud<pcl::PointXYZ> result = *scans.GetRecent(2);
    EXPECT_EQ(static_cast<int>(result.size()), 2);
    EXPECT_TRUE(Equal(result.at(0), expected->at(1)));
    EXPECT_TRUE(Equal(result.at(1), expected->at(2)));
  }

  {
    const pcl::PointCloud<pcl::PointXYZ> result = *scans.GetRecent(4);
    EXPECT_EQ(static_cast<int>(result.size()), 3);
    EXPECT_TRUE(Equal(result.at(0), expected->at(0)));
    EXPECT_TRUE(Equal(result.at(1), expected->at(1)));
    EXPECT_TRUE(Equal(result.at(2), expected->at(2)));
  }
}
