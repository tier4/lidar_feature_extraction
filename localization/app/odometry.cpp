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

#include <memory>

#include "lidar_feature_localization/odometry.hpp"
#include "lidar_feature_localization/point_cloud_map.hpp"
#include "lidar_feature_localization/pose_updater.hpp"
#include "lidar_feature_localization/subscriber.hpp"

using PointToVector = PointXYZCRToVector;
using PointType = typename PointXYZCRToVector::PointType;

using LOAMOdometry = Odometry<
  LOAMPoseUpdater<PointToVector>,
  PointCloudMap<PointType>,
  typename pcl::PointCloud<PointType>::Ptr
>;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto map = std::make_shared<PointCloudMap<PointType>>(7);
  LOAMOdometry odometry(map);

  rclcpp::spin(std::make_shared<OdometrySubscriber<LOAMOdometry, PointType>>(odometry));

  rclcpp::shutdown();
  return 0;
}
