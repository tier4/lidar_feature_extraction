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


#include <gmock/gmock.h>

#include "lidar_feature_library/ros_msg.hpp"

TEST(RosMsg, MakePoint)
{
  Eigen::Vector3d p(2, 4, 6);
  const geometry_msgs::msg::Point q = MakePoint(p);
  EXPECT_EQ(q.x, 2);
  EXPECT_EQ(q.y, 4);
  EXPECT_EQ(q.z, 6);
}

TEST(RosMsg, MakePoseStamped)
{
  const double tx = 1.0;
  const double ty = 2.0;
  const double tz = 3.0;
  const double qw = 1. / std::sqrt(2.);
  const double qx = 0.0;
  const double qy = 1. / std::sqrt(2.);
  const double qz = 0.0;

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(tx, ty, tz);
  pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

  const int32_t seconds = 10000;
  const uint32_t nanoseconds = 20000;
  const rclcpp::Time stamp(seconds, nanoseconds);
  const std::string frame_id = "map";

  const auto msg = MakePoseStamped(pose, stamp, frame_id);

  EXPECT_EQ(msg.pose.position.x, tx);
  EXPECT_EQ(msg.pose.position.y, ty);
  EXPECT_EQ(msg.pose.position.z, tz);

  const double tolerance = 1e-8;
  EXPECT_NEAR(msg.pose.orientation.w, qw, tolerance);
  EXPECT_NEAR(msg.pose.orientation.x, qx, tolerance);
  EXPECT_NEAR(msg.pose.orientation.y, qy, tolerance);
  EXPECT_NEAR(msg.pose.orientation.z, qz, tolerance);

  EXPECT_EQ(msg.header.stamp.sec, seconds);
  EXPECT_EQ(msg.header.stamp.nanosec, nanoseconds);
}

TEST(RosMsg, MakeTransformStamped)
{
  const double tx = 1.0;
  const double ty = 2.0;
  const double tz = 3.0;
  const double qw = 1. / std::sqrt(2.);
  const double qx = 0.0;
  const double qy = 1. / std::sqrt(2.);
  const double qz = 0.0;

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(tx, ty, tz);
  transform.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

  const int32_t seconds = 10000;
  const uint32_t nanoseconds = 20000;
  const rclcpp::Time stamp(seconds, nanoseconds);
  const std::string frame_id = "map";

  const auto msg = MakeTransformStamped(transform, stamp, frame_id);

  EXPECT_EQ(msg.transform.translation.x, tx);
  EXPECT_EQ(msg.transform.translation.y, ty);
  EXPECT_EQ(msg.transform.translation.z, tz);

  const double tolerance = 1e-8;
  EXPECT_NEAR(msg.transform.rotation.w, qw, tolerance);
  EXPECT_NEAR(msg.transform.rotation.x, qx, tolerance);
  EXPECT_NEAR(msg.transform.rotation.y, qy, tolerance);
  EXPECT_NEAR(msg.transform.rotation.z, qz, tolerance);

  EXPECT_EQ(msg.header.stamp.sec, seconds);
  EXPECT_EQ(msg.header.stamp.nanosec, nanoseconds);
}

TEST(RosMsg, Vector3ToVector3d)
{
  const double x = 1.;
  const double y = 2.;
  const double z = 3.;

  geometry_msgs::msg::Vector3 msg;
  msg.x = x;
  msg.y = y;
  msg.z = z;

  const Eigen::Vector3d t = ToVector3d(msg);
  EXPECT_EQ(t.x(), x);
  EXPECT_EQ(t.y(), y);
  EXPECT_EQ(t.z(), z);
}

TEST(RosMsg, PointToVector3d)
{
  const double x = 1.;
  const double y = 2.;
  const double z = 3.;

  geometry_msgs::msg::Point msg;
  msg.x = x;
  msg.y = y;
  msg.z = z;

  const Eigen::Vector3d t = ToVector3d(msg);
  EXPECT_EQ(t.x(), x);
  EXPECT_EQ(t.y(), y);
  EXPECT_EQ(t.z(), z);
}

TEST(RosMsg, ToQuaterniond)
{
  const double qw = std::sqrt(2. / 3.);
  const double qx = std::sqrt(1. / 3.);
  const double qy = 0.;
  const double qz = 0.;

  geometry_msgs::msg::Quaternion msg;
  msg.w = qw;
  msg.x = qx;
  msg.y = qy;
  msg.z = qz;

  const Eigen::Quaterniond q = ToQuaterniond(msg);
  const double tolerance = 1e-8;
  EXPECT_NEAR(q.w(), qw, tolerance);
  EXPECT_NEAR(q.x(), qx, tolerance);
  EXPECT_NEAR(q.y(), qy, tolerance);
  EXPECT_NEAR(q.z(), qz, tolerance);
}
