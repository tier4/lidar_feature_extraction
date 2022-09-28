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
#include <string>

#include "ekf_localizer/pose_measurement.hpp"

#include "ekf_localizer/check.hpp"
#include "ekf_localizer/delay.hpp"
#include "ekf_localizer/matrix_types.hpp"
#include "ekf_localizer/normalize_yaw.hpp"


Eigen::Vector3d GetPoseState(const Vector6d & x)
{
  return x.head(3);
}

Eigen::Matrix3d PoseCovariance(const Eigen::MatrixXd & P)
{
  return P.block(0, 0, 3, 3);
}

Eigen::Matrix<double, 3, 6> PoseMeasurementMatrix()
{
  Eigen::Matrix<double, 3, 6> C = Eigen::Matrix<double, 3, 6>::Zero();
  C(0, 0) = 1.0;    // for pos x
  C(1, 1) = 1.0;    // for pos y
  C(2, 2) = 1.0;  // for yaw
  return C;
}

Eigen::Matrix3d PoseMeasurementCovariance(
  const std::array<double, 36ul> & covariance, const size_t smoothing_step)
{
  Eigen::Matrix3d R;
  R <<
    covariance.at(6 * 0 + 0), covariance.at(6 * 0 + 1), covariance.at(6 * 0 + 5),
    covariance.at(6 * 1 + 0), covariance.at(6 * 1 + 1), covariance.at(6 * 1 + 5),
    covariance.at(6 * 5 + 0), covariance.at(6 * 5 + 1), covariance.at(6 * 5 + 5);
  return R * static_cast<double>(smoothing_step);
}

Eigen::Vector3d PoseMeasurementVector(const geometry_msgs::msg::Pose & pose)
{
  const double yaw = tf2::getYaw(pose.orientation);
  return Eigen::Vector3d(pose.position.x, pose.position.y, normalizeYaw(yaw));
}

void PoseMeasurement::Push(PoseWithCovarianceStamped::SharedPtr msg)
{
  messages_.push(msg);
}

void PoseMeasurement::Clear()
{
  messages_.clear();
}

void PoseMeasurement::Update(
  std::shared_ptr<TimeDelayKalmanFilter> & ekf,
  const rclcpp::Time & current_time,
  const double dt)
{
  const Eigen::Matrix<double, 3, 6> C = PoseMeasurementMatrix();

  for (size_t i = 0; i < messages_.size(); ++i) {
    const auto pose = messages_.pop();

    CheckFrameId(warning_, pose->header.frame_id, frame_id_);

    const double delay_time = ComputeDelayTime(current_time, pose->header.stamp);
    CheckDelayTime(warning_, delay_time);

    const int delay_step = ComputeDelayStep(delay_time, dt);
    if (!CheckDelayStep(warning_, delay_step, extend_state_step_)) {
      continue;
    }

    const Eigen::Vector3d y = PoseMeasurementVector(pose->pose.pose);

    if (!CheckMeasurementMatrixNanInf(warning_, y)) {
      continue;
    }

    const Eigen::Vector3d y_ekf = GetPoseState(ekf->getX(delay_step));
    const Eigen::Matrix3d P_y = PoseCovariance(ekf->getLatestP());

    if (!CheckMahalanobisGate(warning_, gate_dist_, y_ekf, y, P_y)) {
      continue;
    }

    const Eigen::Matrix3d R = PoseMeasurementCovariance(pose->pose.covariance, smoothing_steps_);

    try {
      ekf->update(y, C, R, delay_step);
    } catch (std::invalid_argument & e) {
      warning_.Warn(e.what());
    }
  }
}
