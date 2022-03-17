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

#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "lidar_feature_localization/optimization_problem.hpp"
#include "lidar_feature_localization/optimizer.hpp"
#include "lidar_feature_localization/matrix_type.hpp"
#include "lidar_feature_localization/posevec.hpp"

#include "lidar_feature_library/convert_point_cloud_type.hpp"
#include "lidar_feature_library/point_type.hpp"
#include "lidar_feature_library/ros_msg.hpp"

#include <string>
#include <memory>
#include <limits>


class Localizer
{
public:
  Localizer(
    const typename pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_map,
    const typename pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_map)
  : edge_map_(edge_map),
    surface_map_(surface_map),
    is_initialized_(false),
    pose_(Eigen::Isometry3d::Identity())
  {
  }

  void Init(const Eigen::Isometry3d & initial_pose)
  {
    pose_ = initial_pose;
    is_initialized_ = true;
  }

  bool Run(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_xyz,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_xyz)
  {
    const OptimizationProblem problem(edge_map_, surface_map_);
    if (problem.IsDegenerate(edge_xyz, surface_xyz, pose_)) {
      RCLCPP_WARN(
        rclcpp::get_logger("lidar_feature_localization"),
        "The optimization problem is degenerate. Pose not optimized");
      return false;
    }

    const Optimizer optimizer(problem);
    pose_ = optimizer.Run(std::make_tuple(edge_xyz, surface_xyz), pose_);
    return true;
  }

  Eigen::Isometry3d Get() const
  {
    return pose_;
  }

  bool IsInitialized() const
  {
    return is_initialized_;
  }

private:
  const typename pcl::PointCloud<pcl::PointXYZ>::Ptr edge_map_;
  const typename pcl::PointCloud<pcl::PointXYZ>::Ptr surface_map_;
  bool is_initialized_;
  Eigen::Isometry3d pose_;
};

using Exact = message_filters::sync_policies::ExactTime<
  sensor_msgs::msg::PointCloud2,
  sensor_msgs::msg::PointCloud2>;
using Synchronizer = message_filters::Synchronizer<Exact>;

const rmw_qos_profile_t qos_profile = rclcpp::SensorDataQoS().keep_last(1).get_rmw_qos_profile();

geometry_msgs::msg::Pose MakePose(
  const geometry_msgs::msg::Quaternion & orientation,
  const geometry_msgs::msg::Point & position)
{
  geometry_msgs::msg::Pose pose;
  pose.position = position;
  pose.orientation = orientation;
  return pose;
}

geometry_msgs::msg::PoseStamped MakePoseStamped(
  const Eigen::Isometry3d & pose, const rclcpp::Time & stamp, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped pose_stamped_msg;
  pose_stamped_msg.pose = tf2::toMsg(pose);
  pose_stamped_msg.header.stamp = stamp;
  pose_stamped_msg.header.frame_id = frame_id;
  return pose_stamped_msg;
}

template<typename T>
pcl::PointCloud<pcl::PointXYZ>::Ptr ToPointXYZ(const typename pcl::PointCloud<T>::Ptr & cloud)
{
  return ConvertPointCloudType<T, pcl::PointXYZ>(cloud);
}

class Subscriber : public rclcpp::Node
{
public:
  Subscriber(
    const typename pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_map,
    const typename pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_map)
  : Node("lidar_feature_localization"),
    initial_pose_subscriber_(
      this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "initial_pose", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&Subscriber::PoseInitializationCallback, this, std::placeholders::_1),
        this->MakeInitialSubscriptionOption())),
    pose_publisher_(
      this->create_publisher<geometry_msgs::msg::PoseStamped>("estimated_pose", 10)),
    edge_subscriber_(this, "scan_edge", qos_profile),
    surface_subscriber_(this, "scan_surface", qos_profile),
    sync_(std::make_shared<Synchronizer>(Exact(10), edge_subscriber_, surface_subscriber_)),
    localizer_(std::make_shared<Localizer>(edge_map, surface_map))
  {
    sync_->registerCallback(
      std::bind(
        &Subscriber::PoseUpdateCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Subscriber created");
  }

  rclcpp::SubscriptionOptions MakeInitialSubscriptionOption()
  {
    rclcpp::CallbackGroup::SharedPtr main_callback_group;
    main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto main_sub_opt = rclcpp::SubscriptionOptions();
    main_sub_opt.callback_group = main_callback_group;
    return main_sub_opt;
  }

  void PoseInitializationCallback(
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr initial_pose)
  {
    if (localizer_->IsInitialized()) {
      return;
    }

    Eigen::Isometry3d transform;
    tf2::fromMsg(initial_pose->pose, transform);
    localizer_->Init(transform);

    RCLCPP_INFO(this->get_logger(), "Initialized");
  }

  void PoseUpdateCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr edge_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr surface_msg)
  {
    RCLCPP_INFO(this->get_logger(), "Pose update called");

    const pcl::PointCloud<PointXYZIR>::Ptr edge = getPointCloud<PointXYZIR>(*edge_msg);
    const pcl::PointCloud<PointXYZIR>::Ptr surface = getPointCloud<PointXYZIR>(*surface_msg);

    const pcl::PointCloud<pcl::PointXYZ>::Ptr edge_xyz = ToPointXYZ<PointXYZIR>(edge);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr surface_xyz = ToPointXYZ<PointXYZIR>(surface);

    localizer_->Run(edge_xyz, surface_xyz);
    const Eigen::Isometry3d pose = localizer_->Get();
    pose_publisher_->publish(MakePoseStamped(pose, edge_msg->header.stamp, "map"));

    RCLCPP_INFO(this->get_logger(), "Pose update done");
  }

private:
  const rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_subscriber_;
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> edge_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> surface_subscriber_;
  std::shared_ptr<Synchronizer> sync_;
  std::shared_ptr<Localizer> localizer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  pcl::PointCloud<PointXYZIR>::Ptr edge_map(new pcl::PointCloud<PointXYZIR>());
  pcl::PointCloud<PointXYZIR>::Ptr surface_map(new pcl::PointCloud<PointXYZIR>());
  pcl::io::loadPCDFile("edge.pcd", *edge_map);
  pcl::io::loadPCDFile("surface.pcd", *surface_map);

  const pcl::PointCloud<pcl::PointXYZ>::Ptr edge_map_xyz = ToPointXYZ<PointXYZIR>(edge_map);
  const pcl::PointCloud<pcl::PointXYZ>::Ptr surface_map_xyz = ToPointXYZ<PointXYZIR>(surface_map);
  rclcpp::spin(std::make_shared<Subscriber>(edge_map_xyz, surface_map_xyz));
  rclcpp::shutdown();
  return 0;
}
