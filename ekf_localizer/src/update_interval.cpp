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

#include <fmt/core.h>

#include <stdexcept>

#include "ekf_localizer/update_interval.hpp"


double UpdateInterval::Compute(const double current_time_second)
{
  if (!last_time_.has_value()) {
    last_time_ = std::make_optional<const double>(current_time_second);
    return ComputeInterval(default_frequency_);
  }

  if (current_time_second < last_time_.value()) {
    throw std::invalid_argument(
      fmt::format(
        "Detected jump back in time. "
        "current time = {:10.9f}, last time = {:10.9f}",
        current_time_second, last_time_.value()));
  }

  const double frequency = 1.0 / (current_time_second - last_time_.value());
  last_time_ = std::make_optional<const double>(current_time_second);
  return ComputeInterval(frequency);
}
