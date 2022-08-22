// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "ekf_localizer/tf.hpp"


class EKFLocalizerTestSuite : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

class TfBroadcasterNode : public rclcpp::Node
{
public:
  TfBroadcasterNode()
  : rclcpp::Node("tf_broadcaster"),
    broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
  }

  void Broadcast()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.set__stamp(this->now()).set__frame_id("map");
    transform.child_frame_id = "base_link";
    transform.transform.translation.set__x(10.).set__y(20.).set__z(30.);
    transform.transform.rotation.set__w(1.).set__x(0.).set__y(0.).set__z(0.);

    broadcaster_->sendTransform(transform);
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

TEST_F(EKFLocalizerTestSuite, getTransformFromTF)
{
  auto broadcaster = std::make_shared<TfBroadcasterNode>();

  auto timer = rclcpp::create_timer(
    broadcaster, broadcaster->get_clock(), std::chrono::milliseconds(100),
    std::bind(&TfBroadcasterNode::Broadcast, broadcaster));

  TransformListener listener(broadcaster.get());

  rclcpp::Rate r(10);

  r.sleep();
  rclcpp::spin_some(broadcaster);

  std::optional<geometry_msgs::msg::TransformStamped> maybe_transform;
  for (int i = 0; i < 10; ++i) {
    maybe_transform = listener.LookupTransform("map", "base_link");

    r.sleep();
    rclcpp::spin_some(broadcaster);
  }

  ASSERT_TRUE(maybe_transform.has_value());

  EXPECT_EQ(maybe_transform->transform.translation.x, 10.);
  EXPECT_EQ(maybe_transform->transform.translation.y, 20.);
  EXPECT_EQ(maybe_transform->transform.translation.z, 30.);
  EXPECT_EQ(maybe_transform->transform.rotation.w, 1.);
  EXPECT_EQ(maybe_transform->transform.rotation.x, 0.);
  EXPECT_EQ(maybe_transform->transform.rotation.y, 0.);
  EXPECT_EQ(maybe_transform->transform.rotation.z, 0.);

}
