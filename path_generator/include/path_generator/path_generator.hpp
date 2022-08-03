// Copyright 2022 Takeshi Ishita
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
//    * Neither the name of the Takeshi Ishita nor the names of its
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


#ifndef PATH_GENERATOR__PATH_GENERATOR_HPP_
#define PATH_GENERATOR__PATH_GENERATOR_HPP_

#include <functional>
#include <memory>
#include <string>

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lidar_feature_library/qos.hpp"

class PathGenerator : public rclcpp::Node
{
public:
  explicit PathGenerator(const std::string & path_topic_name)
  : Node("path_generator"),
    path_publisher_(
      this->create_publisher<nav_msgs::msg::Path>(path_topic_name, QOS_RELIABLE_TRANSIENT_LOCAL))
  {
  }

  void Publish(const std_msgs::msg::Header & header, const geometry_msgs::msg::Pose & pose)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.pose = pose;

    path_.header = header;
    path_.poses.push_back(pose_stamped);

    path_publisher_->publish(path_);
  }

private:
  nav_msgs::msg::Path path_;
  const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};

class FromPose : public PathGenerator
{
public:
  using PoseStamped = geometry_msgs::msg::PoseStamped;

  FromPose(const std::string & path_topic_name, const std::string & pose_topic_name)
  : PathGenerator(path_topic_name),
    subscription_(
      this->create_subscription<PoseStamped>(
        pose_topic_name, rclcpp::SensorDataQoS().reliable().durability_volatile().keep_all(),
        std::bind(&FromPose::Callback, this, std::placeholders::_1)))
  {
  }

  void Callback(const PoseStamped::ConstSharedPtr pose)
  {
    this->Publish(pose->header, pose->pose);
  }

private:
  rclcpp::Subscription<PoseStamped>::SharedPtr subscription_;
};

class FromPoseWithCovariance : public PathGenerator
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  FromPoseWithCovariance(const std::string & path_topic_name, const std::string & pose_topic_name)
  : PathGenerator(path_topic_name),
    subscription_(
      this->create_subscription<PoseWithCovarianceStamped>(
        pose_topic_name, rclcpp::SensorDataQoS().reliable().durability_volatile().keep_all(),
        std::bind(&FromPoseWithCovariance::Callback, this, std::placeholders::_1)))
  {
  }

  void Callback(const PoseWithCovarianceStamped::ConstSharedPtr pose)
  {
    this->Publish(pose->header, pose->pose.pose);
  }

private:
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr subscription_;
};

#endif  // PATH_GENERATOR__PATH_GENERATOR_HPP_
