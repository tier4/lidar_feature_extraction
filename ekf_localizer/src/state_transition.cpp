// Copyright 2022 Autoware Foundation
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

#include <cmath>

#include "ekf_localizer/matrix_types.hpp"
#include "ekf_localizer/state_index.hpp"

double normalizeYaw(const double & yaw)
{
  // FIXME(IshitaTakeshi) I think the computation here can be simplified
  // FIXME(IshitaTakeshi) Rename the function. This is not normalization
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

Vector6d predictNextState(const Vector6d & X_curr, const double dt)
{
  const double x = X_curr(IDX::X);
  const double y = X_curr(IDX::Y);
  const double biased_yaw = X_curr(IDX::YAW);
  const double yaw_bias = X_curr(IDX::YAWB);
  const double yaw = biased_yaw + yaw_bias;
  const double vx = X_curr(IDX::VX);
  const double wz = X_curr(IDX::WZ);

  Vector6d X_next;
  X_next(IDX::X) = x + vx * std::cos(yaw) * dt;  // dx = v * cos(yaw)
  X_next(IDX::Y) = y + vx * std::sin(yaw) * dt;  // dy = v * sin(yaw)
  X_next(IDX::YAW) = normalizeYaw(biased_yaw + wz * dt);           // dyaw = omega + omega_bias
  X_next(IDX::YAWB) = yaw_bias;
  X_next(IDX::VX) = vx;
  X_next(IDX::WZ) = wz;
  return X_next;
}

// TODO(TakeshiIshita) show where the equation come from
Matrix6d createStateTransitionMatrix(const Vector6d & X_curr, const double dt)
{
  const double biased_yaw = X_curr(IDX::YAW);
  const double yaw_bias = X_curr(IDX::YAWB);
  const double yaw = biased_yaw + yaw_bias;
  const double vx = X_curr(IDX::VX);

  Matrix6d A = Matrix6d::Identity();
  A(IDX::X, IDX::YAW) = -vx * sin(yaw) * dt;
  A(IDX::X, IDX::YAWB) = -vx * sin(yaw) * dt;
  A(IDX::X, IDX::VX) = cos(yaw) * dt;
  A(IDX::Y, IDX::YAW) = vx * cos(yaw) * dt;
  A(IDX::Y, IDX::YAWB) = vx * cos(yaw) * dt;
  A(IDX::Y, IDX::VX) = sin(yaw) * dt;
  A(IDX::YAW, IDX::WZ) = dt;
  return A;
}

Matrix6d processNoiseCovariance(const Eigen::Vector4d & variances)
{
  Vector6d q;
  q << 0., 0., variances(0), variances(1), variances(2), variances(3);
  return q.asDiagonal();
}
