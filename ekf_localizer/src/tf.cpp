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
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ekf_localizer/tf.hpp"


bool getTransformFromTF(
  const Warning & warning,
  const std::string & parent_frame,
  const std::string & child_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  tf2::BufferCore tf_buffer;
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  for (int i = 0; i < 50; ++i) {
    try {
      transform = tf_buffer.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
      return true;
    } catch (tf2::TransformException & ex) {
      warning.Warn(ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }
  return false;
}
