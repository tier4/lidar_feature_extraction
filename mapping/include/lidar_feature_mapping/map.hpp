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

#ifndef LIDAR_FEATURE_MAPPING__MAP_HPP_
#define LIDAR_FEATURE_MAPPING__MAP_HPP_

#include <pcl/io/auto_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_eigen/tf2_eigen.hpp>

#include <memory>
#include <string>

#include "lidar_feature_library/point_type.hpp"
#include "lidar_feature_library/ros_msg.hpp"
#include "lidar_feature_library/pcl_utils.hpp"


bool PoseDiffIsSufficientlySmall(
  const Eigen::Isometry3d & pose0,
  const Eigen::Isometry3d & pose1,
  const double translation_threshold,
  const double rotation_threshold)
{
  const Eigen::Isometry3d d = pose0.inverse() * pose1;
  const Eigen::Quaterniond dq(d.rotation());
  const Eigen::Vector3d dt = d.translation();
  return dt.norm() < translation_threshold && dq.vec().norm() < rotation_threshold;
}

template<typename T>
class Map
{
public:
  Map()
  : map_ptr_(new pcl::PointCloud<T>()) {}

  void TransformAdd(
    const Eigen::Isometry3d & transform,
    const typename pcl::PointCloud<T>::Ptr & cloud)
  {
    const auto transformed = TransformPointCloud<T>(transform, cloud);
    *map_ptr_ += *transformed;
  }

  bool IsEmpty() const
  {
    return map_ptr_->size() == 0;
  }

  void Save(const std::string & pcd_filename) const
  {
    pcl::io::save(pcd_filename, *map_ptr_);
  }

  typename pcl::PointCloud<T>::Ptr map_ptr_;
};

const double translation_threshold = 1.0;
const double rotation_threshold = 0.1;

template<typename PointType>
class MapBuilder
{
public:
  MapBuilder()
  : map_(std::make_shared<Map<PointType>>()) {}

  void Callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg)
  {
    const Eigen::Isometry3d transform = GetIsometry3d(pose_msg->pose);
    const auto cloud = GetPointCloud<PointType>(*cloud_msg);

    RCLCPP_INFO(
      rclcpp::get_logger("lidar_feature_mapping"),
      "Recieved cloud of size %lu at %d.%d",
      cloud->size(),
      cloud_msg->header.stamp.sec,
      cloud_msg->header.stamp.nanosec);

    const bool pose_diff_small = PoseDiffIsSufficientlySmall(
      prev_transform_, transform, translation_threshold, rotation_threshold
    );

    if (!map_->IsEmpty() && pose_diff_small) {
      return;
    }

    map_->TransformAdd(transform, cloud);
    prev_transform_ = transform;
  }

  void SaveMap(const std::string & pcd_filename) const
  {
    RCLCPP_INFO(
      rclcpp::get_logger("lidar_feature_mapping"),
      "Saving map to %s", pcd_filename.c_str());

    if (map_->IsEmpty()) {
      RCLCPP_WARN(
        rclcpp::get_logger("lidar_feature_mapping"),
        "Map is empty! Quit without exporting to a file");
      return;
    }

    map_->Save(pcd_filename);
  }

  std::shared_ptr<Map<PointType>> map_;
  Eigen::Isometry3d prev_transform_;
};

#endif  // LIDAR_FEATURE_MAPPING__MAP_HPP_
