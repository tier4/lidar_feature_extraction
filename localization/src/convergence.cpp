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
#include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/rclcpp.hpp>
#include <fmt/core.h>

#include <memory>
#include <random>
#include <string>

#include "lidar_feature_library/ros_msg.hpp"
#include "lidar_feature_localization/loam.hpp"
#include "lidar_feature_localization/optimizer.hpp"

const rmw_qos_profile_t qos_profile =
  rclcpp::SensorDataQoS().keep_all().reliable().get_rmw_qos_profile();

using Exact = message_filters::sync_policies::ExactTime<
  sensor_msgs::msg::PointCloud2,
  geometry_msgs::msg::PoseStamped>;
using Synchronizer = message_filters::Synchronizer<Exact>;

template<typename PointToVector, typename PointType>
class ConvergenceAnalysis : public rclcpp::Node
{
public:
  ConvergenceAnalysis(
    const std::string & edge_topic_name,
    const std::string & pose_topic_name,
    const typename pcl::PointCloud<PointType>::Ptr & edge_map)
  : Node("lidar_feature_convergence"),
    edge_subscriber_(this, edge_topic_name, qos_profile),
    pose_subscriber_(this, pose_topic_name, qos_profile),
    sync_(std::make_shared<Synchronizer>(Exact(10), edge_subscriber_, pose_subscriber_)),
    edge_map_(edge_map)
  {
    std::srand(3939);

    sync_->registerCallback(
      std::bind(
        &ConvergenceAnalysis::OnScanObserved, this, std::placeholders::_1, std::placeholders::_2));
  }

  void OnScanObserved(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & edge_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg)
  {
    RCLCPP_INFO(this->get_logger(), "OnScanObserved is called");

    if (std::rand() % 10 >= 1) {
      RCLCPP_INFO(this->get_logger(), "Pass this time");
      return;
    }

    Eigen::Isometry3d pose;
    tf2::fromMsg(pose_msg->pose, pose);

    const auto edge = GetPointCloud<PointType>(*edge_msg);

    using OptimizationProblem = LOAMOptimizationProblem<PointToVector, PointType>;

    const OptimizationProblem problem(edge_map_);

    using OptimizerType = Optimizer<
      OptimizationProblem,
      typename pcl::PointCloud<PointType>::Ptr>;

    const OptimizerType optimizer(problem);

    std::ofstream file;
    file.open(
      fmt::format(
        "convergence/{}-{}.csv",
        pose_msg->header.stamp.sec,
        pose_msg->header.stamp.nanosec));
    file.precision(10);

    for (int Y = -2; Y <= 2; Y += 1) {
      for (int X = -2; X <= 2; X += 1) {
        const double x = 0.1 * X;
        const double y = 0.1 * Y;

        Eigen::Isometry3d initial_pose;
        initial_pose.linear() = pose.rotation();
        initial_pose.translation() = pose.translation() + Eigen::Vector3d(x, y, 0.);
        const Eigen::Isometry3d estimated = optimizer.Run(edge, initial_pose);

        const Eigen::Vector3d t0 = initial_pose.translation();
        const Eigen::Vector3d t1 = estimated.translation();

        RCLCPP_INFO(
          this->get_logger(),
          "x0 = %+.2f y0 = %+.2f x1 = %+.2f y1 = %+.2f",
          t0(0), t0(1), t1(0), t1(1));

        file <<
          t0(0) << "," << t0(1) << "," << t0(2) << "," <<
          t1(0) << "," << t1(1) << "," << t1(2) << std::endl;
      }
    }

    file.close();

    RCLCPP_INFO(this->get_logger(), "Saved the convegrence result");

    const std::string edge_filename = fmt::format(
      "convergence/{}-{}-edge.pcd",
      pose_msg->header.stamp.sec,
      pose_msg->header.stamp.nanosec);
    pcl::io::save(edge_filename, *edge);

    RCLCPP_INFO(this->get_logger(), "Saved scan");
  }

private:
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> edge_subscriber_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_subscriber_;
  std::shared_ptr<Synchronizer> sync_;
  const typename pcl::PointCloud<PointType>::Ptr edge_map_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  pcl::PointCloud<PointXYZCR>::Ptr edge_map(new pcl::PointCloud<PointXYZCR>());
  pcl::io::loadPCDFile("maps/edge.pcd", *edge_map);

  using Convergence = ConvergenceAnalysis<PointXYZCRToVector, PointXYZCR>;
  auto convergence = std::make_shared<Convergence>("scan_edge", "pose", edge_map);
  rclcpp::spin(convergence);
  rclcpp::shutdown();
  return 0;
}
