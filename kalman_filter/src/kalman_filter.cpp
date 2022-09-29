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

#include "kalman_filter/kalman_filter.hpp"

#include <cassert>

#include "kalman_filter/matrix_size.hpp"

#include "lidar_feature_library/numeric.hpp"


KalmanFilter::KalmanFilter(
  const Eigen::VectorXd & x, const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C, const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
  const Eigen::MatrixXd & P)
: x_(x), P_(P), A_(A), B_(B), C_(C), Q_(Q), R_(R)
{
  assert(!hasZeroElements(x));
  assert(!hasZeroElements(P));
  assert(!hasZeroElements(A));
  assert(!hasZeroElements(B));
  assert(!hasZeroElements(C));
  assert(!hasZeroElements(Q));
  assert(!hasZeroElements(R));
}

void KalmanFilter::init(const Eigen::VectorXd & x, const Eigen::MatrixXd & P0)
{
  assert(!hasZeroElements(x));
  assert(!hasZeroElements(P0));
  x_ = x;
  P_ = P0;
}

double KalmanFilter::getXelement(unsigned int i) const {return x_(i);}

void KalmanFilter::predict(
  const Eigen::VectorXd & u, const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & Q)
{
  assert(A.cols() == x_.rows());
  assert(B.cols() == u.rows());
  assert(A.cols() == P_.rows());
  assert(Q.cols() == Q.rows());
  assert(A.rows() == Q.cols());

  x_ = predictNextState(x_, u, A, B);
  P_ = predictNextCovariance(P_, A, Q);
}

void KalmanFilter::predict(const Eigen::VectorXd & u) {return predict(u, A_, B_, Q_);}

void KalmanFilter::update(const Eigen::VectorXd & y)
{
  const Eigen::MatrixXd K = calcKalmanGain(P_, C_, R_);

  if (HasNan(K) || HasInf(K)) {
    throw std::invalid_argument("K has invalid value");
  }

  x_ = updateState(x_, y, C_, K);
  P_ = updateCovariance(P_, C_, K);
}
