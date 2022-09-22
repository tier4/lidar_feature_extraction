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

Eigen::MatrixXd initX(const Eigen::MatrixXd & x0, const int n)
{
  const int d = x0.rows();

  Eigen::MatrixXd x(n * d, 1);
  for (int i = 0; i < n; ++i) {
    x.block(i * d, 0, d, 1) = x0;
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

Eigen::MatrixXd updateX(const Eigen::MatrixXd & x_, const Eigen::MatrixXd & x_next)
{
  const int a = x_.size();
  const int b = x_next.size();
  const int c = a - b;

  Eigen::MatrixXd updated(a, 1);
  updated.block(0, 0, b, 1) = x_next;
  updated.block(b, 0, c, 1) = x_.block(0, 0, c, 1);
  return updated;
}

Eigen::MatrixXd updateP(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q)
{
  /*
   * time delay model:
   *
   *     [A   0   0]      [P11   P12   P13]      [Q   0   0]
   * A = [I   0   0], P = [P21   P22   P23], Q = [0   0   0]
   *     [0   I   0]      [P31   P32   P33]      [0   0   0]
   *
   * covariance calculation in prediction : P = A * P * A' + Q
   *
   *     [A*P11*A'*+Q  A*P11  A*P12]
   * P = [     P11*A'    P11    P12]
   *     [     P21*A'    P21    P22]
   */

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

  Eigen::MatrixXd C_ex = Eigen::MatrixXd::Zero(h, w * max_delay_step);
  C_ex.block(0, w * delay_step, h, w) = C;
  return C_ex;
}

TimeDelayKalmanFilter::TimeDelayKalmanFilter(
  const Eigen::MatrixXd & x, const Eigen::MatrixXd & P0, const int max_delay_step)
: x_(initX(x, max_delay_step)), P_(initP(P0, max_delay_step)),
  max_delay_step_(max_delay_step), dim_x_(x.rows())
{
  assert(x.size() == P0.rows());
}

Eigen::MatrixXd TimeDelayKalmanFilter::getLatestX() const
{
  return x_.block(0, 0, dim_x_, 1);
}

Eigen::MatrixXd TimeDelayKalmanFilter::getLatestP() const
{
  return P_.block(0, 0, dim_x_, dim_x_);
}

bool TimeDelayKalmanFilter::predictWithDelay(
  const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q)
{
  assert(A.rows() == A.cols());
  assert(Q.rows() == Q.cols());
  assert(A.rows() == Q.rows());
  assert(A.rows() == x_next.size());

  x_ = updateX(x_, x_next);
  P_ = updateP(P_, A, Q);

  return true;
}

double TimeDelayKalmanFilter::getXelement(const int delay_step, const int i) const
{
  return x_(delay_step * dim_x_ + i);
}

bool TimeDelayKalmanFilter::updateWithDelay(
  const Eigen::MatrixXd & y, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R,
  const int delay_step)
{
  if (delay_step >= max_delay_step_) {
    std::cerr << "delay step is larger than max_delay_step. ignore update." << std::endl;
    return false;
  }

  assert(C.rows() == y.rows());
  assert(C.cols() == dim_x_);

  /* set measurement matrix */
  const Eigen::MatrixXd C_ex = makeMeasurementMatrix(C, max_delay_step_, delay_step);
  const Eigen::MatrixXd K = calcKalmanGain(P_, C_ex, R);

  if (HasNan(K) || HasInf(K)) {
    return false;
  }

  x_ = updateState(x_, y, C_ex, K);
  P_ = updateCovariance(P_, C_ex, K);

  return true;
}
