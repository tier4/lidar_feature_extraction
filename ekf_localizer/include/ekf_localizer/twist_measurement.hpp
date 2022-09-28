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

#ifndef EKF_LOCALIZER__TWIST_MEASUREMENT_HPP_
#define EKF_LOCALIZER__TWIST_MEASUREMENT_HPP_

#include <tf2/utils.h>

#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "ekf_localizer/aged_message_queue.hpp"
#include "ekf_localizer/check.hpp"
#include "ekf_localizer/delay.hpp"
#include "ekf_localizer/matrix_types.hpp"

#include "kalman_filter/time_delay_kalman_filter.hpp"


using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;


Eigen::Vector2d GetTwistState(const Vector6d & x);
Eigen::Matrix2d TwistCovariance(const Eigen::MatrixXd & P);
Eigen::Matrix<double, 2, 6> TwistMeasurementMatrix();
Eigen::Matrix2d TwistMeasurementCovariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step);
Eigen::Vector2d TwistMeasurementVector(const geometry_msgs::msg::Twist & twist);

class TwistMeasurement
{
public:
  TwistMeasurement(
    const Warning & warning,
    const int extend_state_step,
    const double gate_dist,
    const int smoothing_steps)
  : messages_(smoothing_steps),
    warning_(warning), extend_state_step_(extend_state_step),
    gate_dist_(gate_dist), smoothing_steps_(smoothing_steps)
  {
  }

  void Push(TwistWithCovarianceStamped::SharedPtr msg);
  void Clear();
  void Update(
    std::shared_ptr<TimeDelayKalmanFilter> & ekf,
    const rclcpp::Time & current_time,
    const double dt);

private:
  AgedMessageQueue<TwistWithCovarianceStamped::SharedPtr> messages_;
  const Warning warning_;
  const int extend_state_step_;
  const double gate_dist_;
  const int smoothing_steps_;
};

#endif  // EKF_LOCALIZER__TWIST_MEASUREMENT_HPP_
