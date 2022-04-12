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


#ifndef LIDAR_FEATURE_LIBRARY__ROS_MSG_HPP_
#define LIDAR_FEATURE_LIBRARY__ROS_MSG_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>

#include <string>

template<typename T>
sensor_msgs::msg::PointCloud2 ToRosMsg(const typename pcl::PointCloud<T>::Ptr & cloud_ptr)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud_ptr, msg);
  return msg;
}

template<typename T>
sensor_msgs::msg::PointCloud2 ToRosMsg(
  const typename pcl::PointCloud<T>::Ptr & cloud_ptr,
  const rclcpp::Time & stamp,
  const std::string frame)
{
  sensor_msgs::msg::PointCloud2 msg = ToRosMsg<T>(cloud_ptr);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  return msg;
}

template<typename T>
typename pcl::PointCloud<T>::Ptr GetPointCloud(const sensor_msgs::msg::PointCloud2 & roscloud)
{
  typename pcl::PointCloud<T>::Ptr pclcloud(new pcl::PointCloud<T>());
  pcl::fromROSMsg(roscloud, *pclcloud);
  return pclcloud;
}

Eigen::Affine3d GetAffine(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Affine3d transform;
  tf2::fromMsg(pose, transform);
  return transform;
}

geometry_msgs::msg::TransformStamped EigenToTransform(
  const Eigen::Isometry3d & eigen_transform,
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform = tf2::eigenToTransform(eigen_transform);
  transform.header.stamp = stamp;
  transform.header.frame_id = frame_id;
  transform.child_frame_id = child_frame_id;
  return transform;
}

#endif   // LIDAR_FEATURE_LIBRARY__ROS_MSG_HPP_
