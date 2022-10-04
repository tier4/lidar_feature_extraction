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

#ifndef EKF_LOCALIZER__COVARIANCE_HPP_
#define EKF_LOCALIZER__COVARIANCE_HPP_

#include <Eigen/Core>

#include <array>

#include "ekf_localizer/matrix_types.hpp"

std::array<double, 36> EKFCovarianceToPoseMessageCovariance(const Matrix6d & P);
std::array<double, 36> EKFCovarianceToTwistMessageCovariance(const Matrix6d & P);

#endif  // EKF_LOCALIZER__COVARIANCE_HPP_
