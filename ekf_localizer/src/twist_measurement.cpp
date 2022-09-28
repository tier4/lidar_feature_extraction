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

#include <tf2/utils.h>

#include <memory>

#include "ekf_localizer/twist_measurement.hpp"

#include "ekf_localizer/check.hpp"
#include "ekf_localizer/delay.hpp"
#include "ekf_localizer/matrix_types.hpp"

Eigen::Vector2d GetTwistState(const Vector6d & x)
{
  return x.tail(2);
}

Eigen::Matrix2d TwistCovariance(const Eigen::MatrixXd & P)
{
  return P.block(4, 4, 2, 2);
}

Eigen::Matrix<double, 2, 6> TwistMeasurementMatrix()
{
  Eigen::Matrix<double, 2, 6> C = Eigen::Matrix<double, 2, 6>::Zero();
  C(0, 4) = 1.0;  // for vx
  C(1, 5) = 1.0;  // for wz
  return C;
}

Eigen::Matrix2d TwistMeasurementCovariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step)
{
  Eigen::Matrix2d R;
  R <<
    covariance.at(6 * 0 + 0), covariance.at(6 * 0 + 5),
    covariance.at(6 * 5 + 0), covariance.at(6 * 5 + 5);
  return R * static_cast<double>(smoothing_step);
}

Eigen::Vector2d TwistMeasurementVector(const geometry_msgs::msg::Twist & twist)
{
  return Eigen::Vector2d(twist.linear.x, twist.angular.z);
}

void TwistMeasurement::Push(TwistWithCovarianceStamped::SharedPtr msg)
{
  messages_.push(msg);
}

void TwistMeasurement::Clear()
{
  messages_.clear();
}

void TwistMeasurement::Update(
  std::shared_ptr<TimeDelayKalmanFilter> & ekf,
  const rclcpp::Time & current_time,
  const double dt)
{
  for (size_t i = 0; i < messages_.size(); ++i) {
    const auto twist = messages_.pop();

    CheckFrameId(warning_, twist->header.frame_id, "base_link");

    const double delay_time = ComputeDelayTime(current_time, twist->header.stamp);
    CheckDelayTime(warning_, delay_time);

    const int delay_step = ComputeDelayStep(delay_time, dt);
    if (!CheckDelayStep(warning_, delay_step, extend_state_step_)) {
      continue;
    }

    const Eigen::Vector2d y = TwistMeasurementVector(twist->twist.twist);

    if (!CheckMeasurementMatrixNanInf(warning_, y)) {
      continue;
    }

    const Eigen::Vector2d y_ekf = GetTwistState(ekf->getX(delay_step));
    const Eigen::Matrix2d P_y = TwistCovariance(ekf->getLatestP());

    if (!CheckMahalanobisGate(warning_, gate_dist_, y_ekf, y, P_y)) {
      continue;
    }

    const Eigen::Matrix<double, 2, 6> C = TwistMeasurementMatrix();
    const Eigen::Matrix2d R = TwistMeasurementCovariance(twist->twist.covariance, smoothing_steps_);

    try {
      ekf->update(y, C, R, delay_step);
    } catch (std::invalid_argument & e) {
      warning_.Warn(e.what());
    }
  }
}
