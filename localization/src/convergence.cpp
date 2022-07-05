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

std::ofstream OpenFile(const std::string & filename)
{
  std::ofstream file;
  file.open(filename);
  file.precision(10);
  return file;
}

void WritePose(std::ofstream & file, const Eigen::Isometry3d & p0, const Eigen::Isometry3d & p1)
{
  const Eigen::Vector3d t0 = p0.translation();
  const Eigen::Vector3d t1 = p1.translation();
  const Eigen::Quaterniond q0(p0.rotation());
  const Eigen::Quaterniond q1(p1.rotation());

  file <<
    t0(0) << "," << t0(1) << "," << t0(2) << "," <<
    q0.x() << "," << q0.y() << "," << q0.z() << "," << q0.w() << "," <<
    t1(0) << "," << t1(1) << "," << t1(2) << "," <<
    q1.x() << "," << q1.y() << "," << q1.z() << "," << q1.w() << std::endl;
}

void SaveOrigin(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg)
{
  std::ofstream file = OpenFile(
    fmt::format(
      "convergence/{}-{}.origin.csv",
      pose_msg->header.stamp.sec,
      pose_msg->header.stamp.nanosec));

  const Eigen::Isometry3d pose = GetIsometry3d(pose_msg->pose);

  const Eigen::Vector3d t = pose.translation();
  const Eigen::Quaterniond q(pose.rotation());

  file <<
    t(0) << "," << t(1) << "," << t(2) << "," <<
    q.x() << "," << q.y() << "," << q.z() << "," << q.w() << std::endl;

  file.close();
}

template<typename OptimizerType, typename PointType>
void AnalyzeAndSaveConvergence(
  const OptimizerType optimizer,
  const typename pcl::PointCloud<PointType>::Ptr & edge,
  const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg)
{
  const std::string filename = fmt::format(
    "convergence/{}-{}.convergence.csv",
    pose_msg->header.stamp.sec,
    pose_msg->header.stamp.nanosec);

  std::ofstream file = OpenFile(filename);

  RCLCPP_INFO(
    rclcpp::get_logger("lidar_feature_convergence"),
    "%d.%d",
    pose_msg->header.stamp.sec,
    pose_msg->header.stamp.nanosec);

  Eigen::Isometry3d groundtruth_pose;
  tf2::fromMsg(pose_msg->pose, groundtruth_pose);

  for (int Y = -2; Y <= 2; Y += 1) {
    for (int X = -2; X <= 2; X += 1) {
      const double x = 0.4 * X;
      const double y = 0.4 * Y;

      const Eigen::Vector3d dt(x, y, 0.);

      Eigen::Isometry3d initial_pose;
      initial_pose.linear() = groundtruth_pose.rotation();
      initial_pose.translation() = groundtruth_pose.translation() + dt;
      const Eigen::Isometry3d estimated = optimizer.Run(edge, initial_pose);

      RCLCPP_INFO(
        rclcpp::get_logger("lidar_feature_convergence"),
        "  dx0 = %+.2f dy0 = %+.2f", dt(0), dt(1));

      WritePose(file, initial_pose, estimated);
    }
  }

  file.close();
}

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
    edge_map_(edge_map),
    tf_broadcaster_(*this)
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

    Eigen::Isometry3d pose;
    tf2::fromMsg(pose_msg->pose, pose);

    tf_broadcaster_.sendTransform(
      EigenToTransform(pose, edge_msg->header.stamp, "map", "base_link")
    );

    const auto edge = GetPointCloud<PointType>(*edge_msg);

    using OptimizationProblem = LOAMOptimizationProblem<PointToVector, PointType>;

    const OptimizationProblem problem(edge_map_);

    using OptimizerType = Optimizer<
      OptimizationProblem,
      typename pcl::PointCloud<PointType>::Ptr>;

    const OptimizerType optimizer(problem);

    SaveOrigin(pose_msg);
    AnalyzeAndSaveConvergence<OptimizerType, PointType>(optimizer, edge, pose_msg);

    RCLCPP_INFO(this->get_logger(), "Saved the convegrence result");

    const std::string edge_filename = fmt::format(
      "convergence/{:10d}-{:>09d}.edge.pcd",
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
  tf2_ros::TransformBroadcaster tf_broadcaster_;
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
