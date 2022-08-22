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

#ifndef EKF_LOCALIZER__TF_HPP_
#define EKF_LOCALIZER__TF_HPP_


#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <memory>
#include <string>

#include "ekf_localizer/warning.hpp"


class TransformListener
{
public:
  TransformListener(rclcpp::Node * node)
  : tf_buffer_(std::make_shared<tf2::BufferCore>()),
    listener_(*tf_buffer_, node, false),
    warning_(std::make_shared<Warning>(node))
  {
  }

  std::optional<geometry_msgs::msg::TransformStamped> LookupTransform(
    const std::string & parent_frame,
    const std::string & child_frame) const;

private:
  const std::shared_ptr<tf2::BufferCore> tf_buffer_;
  const tf2_ros::TransformListener listener_;
  const std::shared_ptr<Warning> warning_;
};

#endif  // EKF_LOCALIZER__TF_HPP_
