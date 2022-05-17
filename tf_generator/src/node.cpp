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


#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <functional>
#include <memory>
#include <string>

const rclcpp::QoS qos_keep_all = rclcpp::SensorDataQoS().keep_all().reliable();

class TFGenerator : public rclcpp::Node
{
public:
  TFGenerator()
  : Node("tf_generator"),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this)),
    subscription_(this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "input_pose", qos_keep_all,
        std::bind(&TFGenerator::PublishMapToBaseLink, this, std::placeholders::_1)))
  {
  }

private:
  void PublishMapToBaseLink(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg)
  {
    Eigen::Isometry3d isometry3d;
    tf2::fromMsg(pose_msg->pose, isometry3d);

    geometry_msgs::msg::TransformStamped transform = tf2::eigenToTransform(isometry3d);
    transform.header = pose_msg->header;
    transform.child_frame_id = "base_link";

    tf_broadcaster_->sendTransform(transform);
  }

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFGenerator>());
  rclcpp::shutdown();
  return 0;
}
