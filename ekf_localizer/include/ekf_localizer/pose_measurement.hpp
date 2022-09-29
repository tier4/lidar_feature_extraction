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

#ifndef EKF_LOCALIZER__POSE_MEASUREMENT_HPP_
#define EKF_LOCALIZER__POSE_MEASUREMENT_HPP_

#include <Eigen/Core>

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ekf_localizer/aged_message_queue.hpp"
#include "ekf_localizer/matrix_types.hpp"
#include "ekf_localizer/warning.hpp"

#include "kalman_filter/time_delay_kalman_filter.hpp"


using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;


Eigen::Vector3d PoseMeasurementVector(const geometry_msgs::msg::Pose & pose);
Eigen::Vector3d GetPoseState(const Vector6d & x);
Eigen::Matrix3d PoseCovariance(const Eigen::MatrixXd & P);
Eigen::Matrix<double, 3, 6> PoseMeasurementMatrix();
Eigen::Matrix3d PoseMeasurementCovariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step);

class PoseMeasurement
{
public:
  PoseMeasurement(
    const Warning & warning,
    const std::string & frame_id,
    const int extend_state_step,
    const double gate_dist,
    const int smoothing_steps)
  : messages_(smoothing_steps),
    warning_(warning),
    frame_id_(frame_id), extend_state_step_(extend_state_step),
    gate_dist_(gate_dist), smoothing_steps_(smoothing_steps)
  {
  }

  void Push(PoseWithCovarianceStamped::SharedPtr msg);
  void Clear();
  void Update(
    std::shared_ptr<TimeDelayKalmanFilter> & ekf,
    const rclcpp::Time & current_time,
    const double dt);

private:
  AgedMessageQueue<PoseWithCovarianceStamped::SharedPtr> messages_;
  const Warning warning_;
  const std::string frame_id_;
  const int extend_state_step_;
  const double gate_dist_;
  const int smoothing_steps_;
};

#endif  // EKF_LOCALIZER__POSE_MEASUREMENT_HPP_
