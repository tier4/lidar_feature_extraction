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

#ifndef EKF_LOCALIZER__CHECK_HPP_
#define EKF_LOCALIZER__CHECK_HPP_

#include <Eigen/Core>

#include <string>

#include "ekf_localizer/mahalanobis.hpp"
#include "ekf_localizer/numeric.hpp"
#include "ekf_localizer/warning.hpp"

void ShowDelayTimeWarning(const Warning & warning, const double delay_time);

void ShowDelayStepWarning(const Warning & warning, const int delay_step, const int state_step);

void ShowFrameIdWarning(
  const Warning & warning,
  const std::string & header_frame_id,
  const std::string & expected_frame_id);

void ShowMahalanobisGateWarning(const Warning & warning);

void ShowMeasurementMatrixNanInfWarning(const Warning & warning);

bool CheckFrameId(
  const Warning & warning,
  const std::string & frame_id,
  const std::string & expected_frame_id);

bool CheckDelayTime(const Warning & warning, const double delay_time);

bool CheckDelayStep(const Warning & warning, const int delay_step, const int max_delay_step);

bool CheckMeasurementMatrixNanInf(const Warning & warning, const Eigen::MatrixXd & M);

bool CheckMahalanobisGate(
  const Warning & warning,
  const double & dist_max,
  const Eigen::MatrixXd & x1,
  const Eigen::MatrixXd & x2,
  const Eigen::MatrixXd & cov);

#endif  // EKF_LOCALIZER__CHECK_HPP_
