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

#ifndef EKF_LOCALIZER__NORMALIZE_YAW_HPP_
#define EKF_LOCALIZER__NORMALIZE_YAW_HPP_


// Noramlizes the yaw angle so that it fits in the range (-pi, pi)
/**
* @brief normalize yaw angle
* @param yaw yaw angle
* @return normalized yaw
*/
inline double normalizeYaw(const double & yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}


#endif  // EKF_LOCALIZER__NORMALIZE_YAW_HPP_
