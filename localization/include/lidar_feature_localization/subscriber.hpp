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

#ifndef LIDAR_FEATURE_LOCALIZATION__SUBSCRIBER_HPP_
#define LIDAR_FEATURE_LOCALIZATION__SUBSCRIBER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "lidar_feature_library/point_type.hpp"
#include "lidar_feature_library/qos.hpp"
#include "lidar_feature_library/ros_msg.hpp"

inline rclcpp::SubscriptionOptions MutuallyExclusiveOption(rclcpp::Node & node)
{
  rclcpp::CallbackGroup::SharedPtr main_callback_group;
  main_callback_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;
  return main_sub_opt;
}

template<typename LocalizerT, typename PointType>
class LocalizationSubscriber : public rclcpp::Node
{
public:
  explicit LocalizationSubscriber(LocalizerT & localizer)
  : Node("lidar_feature_localization"),
    optimization_start_odom_subscriber_(
      this->create_subscription<nav_msgs::msg::Odometry>(
        "optimization_start_odom", QOS_RELIABLE_VOLATILE,
        std::bind(
          &LocalizationSubscriber::OptimizationStartOdomCallback, this, std::placeholders::_1),
        MutuallyExclusiveOption(*this))),
    optimization_start_pose_subscriber_(
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "optimization_start_pose", QOS_RELIABLE_VOLATILE,
        std::bind(
          &LocalizationSubscriber::OptimizationStartPoseCallback, this, std::placeholders::_1),
        MutuallyExclusiveOption(*this))),
    edge_subscriber_(
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "scan_edge", QOS_RELIABLE_VOLATILE,
        std::bind(&LocalizationSubscriber::PoseUpdateCallback, this, std::placeholders::_1),
        MutuallyExclusiveOption(*this))),
    pose_publisher_(
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("estimated_pose", 10)),
    localizer_(localizer),
    tf_broadcaster_(*this)
  {
    RCLCPP_INFO(this->get_logger(), "LocalizationSubscriber created");
  }

  void OptimizationStartOdomCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr optimization_start_odom)
  {
    this->SetOptimizationStartPose(optimization_start_odom->pose.pose);
  }

  void OptimizationStartPoseCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr optimization_start_pose)
  {
    this->SetOptimizationStartPose(optimization_start_pose->pose);
  }

  void SetOptimizationStartPose(const geometry_msgs::msg::Pose & pose_msg)
  {
    const Eigen::Isometry3d pose = GetIsometry3d(pose_msg);
    localizer_.Init(pose);
  }

  void PoseUpdateCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr edge_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Pose update called");

    const auto edge = GetPointCloud<PointType>(*edge_msg);

    localizer_.Update(edge);

    const Eigen::Isometry3d pose = localizer_.Get();
    const RowMatrix6d covariance = RowMatrix6d::Identity();
    pose_publisher_->publish(
      MakePoseWithCovarianceStamped(pose, covariance, edge_msg->header.stamp, "map")
    );

    tf_broadcaster_.sendTransform(
      MakeTransformStamped(pose, edge_msg->header.stamp, "map", "base_link")
    );

    RCLCPP_INFO(this->get_logger(), "Pose update done");
  }

private:
  using Odometry = nav_msgs::msg::Odometry;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  const rclcpp::Subscription<Odometry>::SharedPtr optimization_start_odom_subscriber_;
  const rclcpp::Subscription<PoseStamped>::SharedPtr optimization_start_pose_subscriber_;
  const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr edge_subscriber_;
  const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  LocalizerT localizer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

template<typename OdometryT, typename PointType>
class OdometrySubscriber : public rclcpp::Node
{
public:
  explicit OdometrySubscriber(OdometryT & odometry)
  : Node("lidar_feature_odometry"),
    edge_subscriber_(
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "scan_edge", QOS_RELIABLE_VOLATILE,
        std::bind(&OdometrySubscriber::PoseUpdateCallback, this, std::placeholders::_1),
        MutuallyExclusiveOption(*this))),
    pose_publisher_(
      this->create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose", 10)),
    odometry_(odometry),
    tf_broadcaster_(*this)
  {
    RCLCPP_INFO(this->get_logger(), "LocalizationSubscriber created");
  }

  void PoseUpdateCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr edge_msg)
  {
    RCLCPP_INFO(this->get_logger(), "PoseUpdateCallback called");

    const auto edge_scan = GetPointCloud<PointType>(*edge_msg);

    RCLCPP_INFO(this->get_logger(), "Call odometry update");

    odometry_.Update(edge_scan);

    const Eigen::Isometry3d pose = odometry_.CurrentPose();
    pose_publisher_->publish(MakePoseStamped(pose, edge_msg->header.stamp, "map"));

    tf_broadcaster_.sendTransform(
      MakeTransformStamped(pose, edge_msg->header.stamp, "map", "base_link")
    );

    RCLCPP_INFO(this->get_logger(), "PoseUpdateCallback finished");
  }

private:
  const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr edge_subscriber_;
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  OdometryT odometry_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__SUBSCRIBER_HPP_
