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

#ifndef EKF_LOCALIZER__UPDATE_INTERVAL_HPP_
#define EKF_LOCALIZER__UPDATE_INTERVAL_HPP_

#include <algorithm>
#include <optional>


inline double ComputeInterval(double frequency)
{
  return 1.0 / std::max(frequency, 0.1);
}

// TODO(IshitaTakeshi) Make the constructor take the interval instead of
// frequency becasue it may cause confusion that the constructor takes
// the frequency [1/s] while the compute method takes the time [s]
class UpdateInterval
{
 public:
  UpdateInterval(const double default_frequency)
  : default_frequency_(default_frequency), last_time_(std::nullopt)
  {
  }

  double Compute(const double current_time_second);

 private:
  const double default_frequency_;
  std::optional<double> last_time_;
};

#endif  // EKF_LOCALIZER__UPDATE_INTERVAL_HPP_
