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

#include "kalman_filter/time_delay_kalman_filter.hpp"

#include "lidar_feature_library/numeric.hpp"

Eigen::VectorXd initX(const Eigen::VectorXd & x0, const int n)
{
  const int d = x0.rows();

  Eigen::VectorXd x(n * d, 1);
  for (int i = 0; i < n; ++i) {
    x(Eigen::seqN(i * d, d)) = x0;
  }
  return x;
}

Eigen::MatrixXd initP(const Eigen::MatrixXd & P0, const int n)
{
  assert(P0.rows() == P0.cols());
  const int d = P0.rows();

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n * d, n * d);
  for (int i = 0; i < n; ++i) {
    P.block(i * d, i * d, d, d) = P0;
  }
  return P;
}

Eigen::VectorXd updateX(const Eigen::VectorXd & x, const Eigen::VectorXd & x_next)
{
  const int a = x.size();
  const int b = x_next.size();
  const int c = a - b;

  Eigen::VectorXd updated(a);
  updated(Eigen::seqN(0, b)) = x_next;
  updated(Eigen::seqN(b, c)) = x(Eigen::seqN(0, c));
  return updated;
}

Eigen::MatrixXd updateP(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q)
{
  // P = expectation(
  //     [(A * hat(x3) + b) - (A * x3 + b + w),  hat(x3) - x3,  hat(x2) - x2] *
  //     [(A * hat(x3) + b) - (A * x3 + b + w),  hat(x3) - x3,  hat(x2) - x2]^T
  // )
  /*
   * time delay model:
   *
   *     [A   0   0]      [P11   P12   P13]      [Q   0   0]
   * A = [I   0   0], P = [P21   P22   P23], Q = [0   0   0]
   *     [0   I   0]      [P31   P32   P33]      [0   0   0]
   *
   * covariance calculation in prediction : P = A * P * A' + Q
   *
   *     [A*P11*A'+Q  A*P11  A*P12]
   * P = [    P11*A'    P11    P12]
   *     [    P21*A'    P21    P22]
   */

  assert(A.rows() == A.cols());
  assert(Q.rows() == Q.cols());

  const int a = P.rows();
  const int b = A.rows();
  const int c = a - b;

  const Eigen::MatrixXd BB = P.block(0, 0, b, b);
  const Eigen::MatrixXd BC = P.block(0, 0, b, c);
  const Eigen::MatrixXd CB = P.block(0, 0, c, b);
  const Eigen::MatrixXd CC = P.block(0, 0, c, c);

  Eigen::MatrixXd updated(a, a);
  updated.block(0, 0, b, b) = A * BB * A.transpose() + Q;
  updated.block(0, b, b, c) = A * BC;
  updated.block(b, 0, c, b) = CB * A.transpose();
  updated.block(b, b, c, c) = CC;
  return updated;
}

Eigen::MatrixXd makeMeasurementMatrix(
  const Eigen::MatrixXd & C, const int max_delay_step, const int delay_step)
{
  const int w = C.cols();
  const int h = C.rows();

  Eigen::MatrixXd D = Eigen::MatrixXd::Zero(h, w * max_delay_step);
  D.block(0, w * delay_step, h, w) = C;
  return D;
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> PredictWithDelay(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  const Eigen::VectorXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q)
{
  const Eigen::VectorXd x1 = updateX(x0, x_next);
  const Eigen::MatrixXd P1 = updateP(P0, A, Q);
  return std::make_tuple(x1, P1);
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> UpdateWithDelay(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  const Eigen::VectorXd & y, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R,
  const int delay_step, const int max_delay_step_)
{
  if (delay_step >= max_delay_step_) {
    throw std::invalid_argument("The delay step is larger than the maximum allowed value");
  }

  assert(C.rows() == y.size());

  /* set measurement matrix */
  const Eigen::MatrixXd D = makeMeasurementMatrix(C, max_delay_step_, delay_step);
  const Eigen::MatrixXd K = calcKalmanGain(P0, D, R);

  if (HasNan(K) || HasInf(K)) {
    throw std::invalid_argument("The kalman gain contains nan or inf");
  }

  const Eigen::VectorXd x1 = updateState(x0, y, D, K);
  const Eigen::MatrixXd P1 = updateCovariance(P0, D, K);

  return std::make_tuple(x1, P1);
}

TimeDelayKalmanFilter::TimeDelayKalmanFilter(
  const Eigen::VectorXd & x, const Eigen::MatrixXd & P, const int max_delay_step)
: x_(initX(x, max_delay_step)), P_(initP(P, max_delay_step)),
  max_delay_step_(max_delay_step), dim_x_(x.rows())
{
  assert(x.size() == P.rows());
}

Eigen::VectorXd TimeDelayKalmanFilter::getLatestX() const
{
  return x_(Eigen::seqN(0, dim_x_));
}

Eigen::MatrixXd TimeDelayKalmanFilter::getLatestP() const
{
  return P_.block(0, 0, dim_x_, dim_x_);
}

void TimeDelayKalmanFilter::predict(
  const Eigen::VectorXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q)
{
  assert(A.rows() == Q.rows());
  assert(A.rows() == x_next.size());
  std::tie(x_, P_) = PredictWithDelay(x_, P_, x_next, A, Q);
}

Eigen::VectorXd TimeDelayKalmanFilter::getX(const int delay_step) const
{
  return x_(Eigen::seqN(delay_step * dim_x_, dim_x_));
}

void TimeDelayKalmanFilter::update(
  const Eigen::VectorXd & y, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R,
  const int delay_step)
{
  std::tie(x_, P_) = UpdateWithDelay(x_, P_, y, C, R, delay_step, max_delay_step_);
}
