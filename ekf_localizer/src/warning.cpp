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

#include "ekf_localizer/warning.hpp"


void ShowDelayTimeWarning(const Warning & warning, const double delay_time)
{
  warning.WarnThrottle(
    1000,
    fmt::format(
      "The time stamp is inappropriate (delay = {} [s]), set delay to 0[s].",
      delay_time));
}

void ShowDelayStepWarning(
  const Warning & warning,
  const double delay_time,
  const double extend_state_step,
  const double ekf_dt)
{
  warning.WarnThrottle(
    1000,
    fmt::format(
      "The delay time ({}[s]) exceeds the compensation limit ({}[s]).",
      delay_time, extend_state_step * ekf_dt));
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
