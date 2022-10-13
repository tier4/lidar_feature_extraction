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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <inttypes.h>

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

#include "lidar_feature_localization/stamp_sorted_objects.hpp"


inline rclcpp::SubscriptionOptions MutuallyExclusiveOption(rclcpp::Node & node)
{
  rclcpp::CallbackGroup::SharedPtr main_callback_group;
  main_callback_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = main_callback_group;
  return main_sub_opt;
}

double Nanoseconds(const rclcpp::Time & t)
{
  return static_cast<double>(t.nanoseconds());
}

using Exact = message_filters::sync_policies::ExactTime<
  sensor_msgs::msg::PointCloud2,
  sensor_msgs::msg::PointCloud2>;
using Synchronizer = message_filters::Synchronizer<Exact>;

const rclcpp::QoS qos_keep_all = rclcpp::SensorDataQoS().keep_all().reliable();

template<typename LocalizerT, typename PointType>
class LocalizationSubscriber : public rclcpp::Node
{
public:
  explicit LocalizationSubscriber(LocalizerT & localizer)
  : Node("lidar_feature_localization"),
    localizer_(localizer),
    edge_subscriber_(this, "scan_edge", qos_keep_all.get_rmw_qos_profile()),
    surface_subscriber_(this, "scan_surface", qos_keep_all.get_rmw_qos_profile()),
    sync_(std::make_shared<Synchronizer>(Exact(10), edge_subscriber_, surface_subscriber_)),
    tf_broadcaster_(*this),
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
    pose_publisher_(
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("estimated_pose", 10))
  {
    sync_->registerCallback(
      std::bind(
        &LocalizationSubscriber::PoseUpdateCallback, this,
        std::placeholders::_1, std::placeholders::_2));
  }

  void OptimizationStartOdomCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr odom)
  {
    this->SetOptimizationStartPose(odom->header.stamp, odom->pose.pose);
  }

  void OptimizationStartPoseCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr stamped_pose)
  {
    this->SetOptimizationStartPose(stamped_pose->header.stamp, stamped_pose->pose);
  }

  void SetOptimizationStartPose(const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & pose)
  {
    prior_poses_.Insert(Nanoseconds(stamp), GetIsometry3d(pose));
  }

  void PoseUpdateCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr edge_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr surface_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Pose update called");

    if (prior_poses_.Size() == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Received an edge message but there's no pose in the prior queue");
      return;
    }

    const auto edge = GetPointCloud<PointType>(*edge_msg);
    const auto surface = GetPointCloud<PointType>(*surface_msg);

    const double msg_stamp_nanosec = Nanoseconds(edge_msg->header.stamp);
    const auto [prior_stamp_nanosec, prior] = prior_poses_.GetClosest(msg_stamp_nanosec);
    prior_poses_.RemoveOlderThan(msg_stamp_nanosec + 1e9);  // 1e9 msg_stamp_nanosec = 1s

    RCLCPP_INFO(
      this->get_logger(),
      "Received scan message of time %lf", msg_stamp_nanosec / 1e9);
    RCLCPP_INFO(
      this->get_logger(),
      "Obtained a prior pose of time %lf", prior_stamp_nanosec / 1e9);
    localizer_.Init(prior);
    localizer_.Update(std::make_tuple(edge, surface));

    const Eigen::Isometry3d pose = localizer_.Get();
    Matrix6d covariance;
    covariance <<
      1.0, 0, 0, 0, 0, 0,
      0, 1.0, 0, 0, 0, 0,
      0, 0, 1.0, 0, 0, 0,
      0, 0, 0, 0.1, 0, 0,
      0, 0, 0, 0, 0.1, 0,
      0, 0, 0, 0, 0, 0.1;

    pose_publisher_->publish(
      MakePoseWithCovarianceStamped(pose, covariance, edge_msg->header.stamp, "map")
    );

    tf_broadcaster_.sendTransform(
      MakeTransformStamped(pose, edge_msg->header.stamp, "map", "lidar_feature_base_link")
    );

    RCLCPP_INFO(this->get_logger(), "Pose update done");
  }

private:
  using Odometry = nav_msgs::msg::Odometry;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  LocalizerT localizer_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> edge_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> surface_subscriber_;
  std::shared_ptr<Synchronizer> sync_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  const rclcpp::Subscription<Odometry>::SharedPtr optimization_start_odom_subscriber_;
  const rclcpp::Subscription<PoseStamped>::SharedPtr optimization_start_pose_subscriber_;
  const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  StampSortedObjects<Eigen::Isometry3d> prior_poses_;
};

/*
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
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr edge_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr surface_msg)
  {
    RCLCPP_INFO(this->get_logger(), "PoseUpdateCallback called");
    RCLCPP_INFO(this->get_logger(), "Call odometry update");

    const auto edge_scan = GetPointCloud<PointType>(*edge_msg);
    const auto surface_scan = GetPointCloud<PointType>(*surface_msg);
    odometry_.Update(std::make_tuple(edge_scan, surface_scan));

    const Eigen::Isometry3d pose = odometry_.CurrentPose();
    pose_publisher_->publish(MakePoseStamped(pose, edge_msg->header.stamp, "map"));

    tf_broadcaster_.sendTransform(
      MakeTransformStamped(pose, edge_msg->header.stamp, "map", "lidar_feature_base_link")
    );

    RCLCPP_INFO(this->get_logger(), "PoseUpdateCallback finished");
  }

private:
  const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr edge_subscriber_;
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  OdometryT odometry_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};
*/

#endif  // LIDAR_FEATURE_LOCALIZATION__SUBSCRIBER_HPP_
