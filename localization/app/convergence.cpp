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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl/io/auto_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rclcpp/rclcpp.hpp>

#include <fmt/core.h>

#include <memory>
#include <string>

#include "lidar_feature_library/qos.hpp"
#include "lidar_feature_library/ros_msg.hpp"

#include "lidar_feature_localization/loam.hpp"
#include "lidar_feature_localization/optimizer.hpp"


using Exact = message_filters::sync_policies::ExactTime<
  sensor_msgs::msg::PointCloud2,
  geometry_msgs::msg::PoseStamped>;
using Synchronizer = message_filters::Synchronizer<Exact>;

template<typename OptimizerType, typename PointType>
visualization_msgs::msg::Marker AnalyzeConvergence(
  const OptimizerType optimizer,
  const typename pcl::PointCloud<PointType>::Ptr & edge,
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg)
{
  const Eigen::Isometry3d groundtruth_pose = GetIsometry3d(pose_msg->pose);

  visualization_msgs::msg::Marker lines = InitLines(pose_msg->header.stamp, "map");
  SetColor(lines, 0.8, 0.8, 1.0, 1.);
  SetWidth(lines, 0.02);

  for (int Y = -1; Y <= 1; Y += 1) {
    for (int X = -1; X <= 1; X += 1) {
      const double x = 0.8 * X;
      const double y = 0.8 * Y;

      RCLCPP_INFO(rclcpp::get_logger("lidar_feature_convergence"), "  dx = %+.2f dy = %+.2f", x, y);

      const Eigen::Vector3d dt(x, y, 0.);

      Eigen::Isometry3d initial_pose;
      initial_pose.linear() = groundtruth_pose.rotation();
      initial_pose.translation() = groundtruth_pose.translation() + dt;
      const Eigen::Isometry3d estimated = optimizer.Run(edge, initial_pose);
      AddLine(lines, initial_pose.translation(), estimated.translation());
    }
  }

  return lines;
}

template<typename PointToVector, typename PointType>
class ConvergenceAnalysis : public rclcpp::Node
{
public:
  ConvergenceAnalysis(
    const std::string & edge_topic_name,
    const std::string & pose_topic_name,
    const std::string & marker_topic_name,
    const typename pcl::PointCloud<PointType>::Ptr & edge_map)
  : Node("lidar_feature_convergence"),
    edge_subscriber_(this, edge_topic_name, QOS_RELIABLE_VOLATILE.get_rmw_qos_profile()),
    pose_subscriber_(this, pose_topic_name, QOS_RELIABLE_VOLATILE.get_rmw_qos_profile()),
    sync_(std::make_shared<Synchronizer>(Exact(10), edge_subscriber_, pose_subscriber_)),
    marker_publisher_(
      this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_name, 10)),
    edge_map_(edge_map),
    tf_broadcaster_(*this)
  {
    sync_->registerCallback(
      std::bind(
        &ConvergenceAnalysis::OnScanObserved, this, std::placeholders::_1, std::placeholders::_2));
  }

  void OnScanObserved(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & edge_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg)
  {
    RCLCPP_INFO(this->get_logger(), "OnScanObserved is called");

    RCLCPP_INFO(
      rclcpp::get_logger("lidar_feature_convergence"),
      "%d.%d", pose_msg->header.stamp.sec, pose_msg->header.stamp.nanosec);

    const Eigen::Isometry3d pose = GetIsometry3d(pose_msg->pose);

    const auto transform = MakeTransformStamped(pose, edge_msg->header.stamp, "map", "base_link");
    tf_broadcaster_.sendTransform(transform);

    using OptimizationProblem = LOAMOptimizationProblem<PointToVector, PointType>;
    using OptimizerType = Optimizer<OptimizationProblem, typename pcl::PointCloud<PointType>::Ptr>;

    const OptimizationProblem problem(edge_map_);
    const OptimizerType optimizer(problem);

    const auto edge = GetPointCloud<PointType>(*edge_msg);
    const auto lines = AnalyzeConvergence<OptimizerType, PointType>(optimizer, edge, pose_msg);

    marker_publisher_->publish(lines);
  }

private:
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> edge_subscriber_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_subscriber_;
  std::shared_ptr<Synchronizer> sync_;
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  const typename pcl::PointCloud<PointType>::Ptr edge_map_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // TODO(IshitaTakeshi) set path as a ros parameter
  pcl::PointCloud<PointXYZCR>::Ptr edge_map(new pcl::PointCloud<PointXYZCR>());
  pcl::io::loadPCDFile("maps/edge.pcd", *edge_map);

  using Convergence = ConvergenceAnalysis<PointXYZCRToVector, PointXYZCR>;
  auto convergence = std::make_shared<Convergence>(
    "scan_edge", "pose", "convergence_marker", edge_map);
  rclcpp::spin(convergence);
  rclcpp::shutdown();
  return 0;
}
