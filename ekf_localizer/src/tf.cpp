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


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ekf_localizer/tf.hpp"

std::optional<geometry_msgs::msg::TransformStamped> TransformListener::LookupTransform(
  const std::string & parent_frame,
  const std::string & child_frame) const
{
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  for (int i = 0; i < 50; ++i) {
    try {
      auto transform = tf_buffer_->lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
      return std::make_optional<geometry_msgs::msg::TransformStamped>(transform);
    } catch (tf2::TransformException & ex) {
      warning_->Warn(ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }
  return std::nullopt;
}
