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

#define FMT_HEADER_ONLY
#include <fmt/format.h>

#include "ekf_localizer/check.hpp"


void ShowDelayTimeWarning(const Warning & warning, const double delay_time)
{
  warning.WarnThrottle(
    1000,
    fmt::format(
      "The time stamp is inappropriate (delay = {} [s]), set delay to 0[s].",
      delay_time));
}

void ShowDelayStepWarning(const Warning & warning, const int delay_step, const int state_step)
{
  warning.WarnThrottle(
    1000,
    fmt::format(
      "The delay step {} should be less than the maximum state step {}.",
      delay_step, state_step));
}

void ShowFrameIdWarning(
  const Warning & warning,
  const std::string & header_frame_id,
  const std::string & expected_frame_id)
{
  warning.WarnThrottle(
    2000,
    fmt::format("frame_id is {} while {} is expected", header_frame_id, expected_frame_id));
}

// The message is modified from the original one to improve reusability
void ShowMahalanobisGateWarning(const Warning & warning)
{
  warning.WarnThrottle(
    2000,
    "[EKF] Mahalanobis distance is over limit. Ignore measurement data.");
}

void ShowMeasurementMatrixNanInfWarning(const Warning & warning)
{
  warning.Warn("[EKF] The measurement matrix includes NaN or Inf.");
}

bool CheckFrameId(
  const Warning & warning,
  const std::string & frame_id,
  const std::string & expected_frame_id)
{
  const bool good = frame_id == expected_frame_id;
  if (!good) {
    ShowFrameIdWarning(warning, frame_id, expected_frame_id);
  }
  return good;
}

bool CheckDelayTime(const Warning & warning, const double delay_time)
{
  const bool good = delay_time >= 0.0;
  if (!good) {
    ShowDelayTimeWarning(warning, delay_time);
  }
  return good;
}

bool CheckDelayStep(const Warning & warning, const int delay_step, const int max_delay_step)
{
  const bool good = delay_step < max_delay_step;
  if (!good) {
    ShowDelayStepWarning(warning, delay_step, max_delay_step);
  }
  return good;
}

bool CheckMeasurementMatrixNanInf(const Warning & warning, const Eigen::MatrixXd & M)
{
  const bool good = !HasNan(M) && !HasInf(M);

  if (!good) {
    ShowMeasurementMatrixNanInfWarning(warning);
  }
  return good;
}

bool CheckMahalanobisGate(
  const Warning & warning,
  const double & dist_max,
  const Eigen::MatrixXd & x1,
  const Eigen::MatrixXd & x2,
  const Eigen::MatrixXd & cov)
{
  const bool good = MahalanobisGate(dist_max, x1, x2, cov);
  if (!good) {
    ShowMahalanobisGateWarning(warning);
  }
  return good;
}
