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

#include <gmock/gmock.h>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <chrono>
#include <thread>

#include "path_generator/path_generator.hpp"

class PathEvaluator
{
public:
  PathEvaluator()
  : expected_size_(0)
  {
  }

  void Callback(const nav_msgs::msg::Path::ConstSharedPtr path)
  {
    expected_size_ += 1;

    const size_t size = path->poses.size();
    const auto pose = path->poses.at(size - 1);
    ASSERT_EQ(size, expected_size_);
    EXPECT_EQ(pose.pose.position.x, static_cast<double>(expected_size_));
  }

  size_t expected_size_;
};

TEST(PathGenerator, NodeIntegrationTest)
{
  rclcpp::init(0, {});

  const rclcpp::QoS qos_keep_all = rclcpp::SensorDataQoS().keep_all().reliable();

  const size_t n_spin = 100;

  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto evaluator = std::make_shared<PathEvaluator>();

  auto path_subscription = node->create_subscription<nav_msgs::msg::Path>(
    "path", qos_keep_all,
    std::bind(&PathEvaluator::Callback, evaluator, std::placeholders::_1));
  auto pose_publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "pose", qos_keep_all);

  auto generator = std::make_shared<PathGenerator>("path", "pose");

  rclcpp::ExecutorOptions options;

  rclcpp::executors::MultiThreadedExecutor executor(options);
  executor.add_node(node);
  executor.add_node(generator);
  for (size_t i = 0; i < n_spin; i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.set__x(i + 1);
    pose_publisher->publish(pose);
    executor.spin_once(std::chrono::milliseconds(10));
  }

  EXPECT_THAT(evaluator->expected_size_, testing::Gt(0U));
  rclcpp::shutdown();
}
