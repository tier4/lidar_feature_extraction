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


KalmanFilter::KalmanFilter() {}
KalmanFilter::KalmanFilter(
  const Eigen::MatrixXd & x, const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C, const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
  const Eigen::MatrixXd & P)
{
  init(x, A, B, C, Q, R, P);
}

KalmanFilter::~KalmanFilter() {}
bool KalmanFilter::init(
  const Eigen::MatrixXd & x, const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & C, const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
  const Eigen::MatrixXd & P)
{
  assert(!hasZeroElements(x));
  assert(!hasZeroElements(A));
  assert(!hasZeroElements(B));
  assert(!hasZeroElements(C));
  assert(!hasZeroElements(Q));
  assert(!hasZeroElements(R));
  assert(!hasZeroElements(P));

  x_ = x;
  A_ = A;
  B_ = B;
  C_ = C;
  Q_ = Q;
  R_ = R;
  P_ = P;
  return true;
}
bool KalmanFilter::init(const Eigen::MatrixXd & x, const Eigen::MatrixXd & P0)
{
  assert(!hasZeroElements(x));
  assert(!hasZeroElements(P0));
  x_ = x;
  P_ = P0;
  return true;
}

double KalmanFilter::getXelement(unsigned int i) const {return x_(i);}

bool KalmanFilter::predict(
  const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A, const Eigen::MatrixXd & Q)
{
  assert(x_.rows() == x_next.rows());
  assert(A.cols() == P_.rows());
  assert(Q.cols() == Q.rows());
  assert(A.rows() == Q.cols());

  x_ = x_next;
  P_ = A * P_ * A.transpose() + Q;
  return true;
}
bool KalmanFilter::predict(const Eigen::MatrixXd & x_next, const Eigen::MatrixXd & A)
{
  return predict(x_next, A, Q_);
}

bool KalmanFilter::predict(
  const Eigen::MatrixXd & u, const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
  const Eigen::MatrixXd & Q)
{
  assert(A.cols() == x_.rows());
  assert(B.cols() == u.rows());
  const Eigen::MatrixXd x_next = A * x_ + B * u;
  return predict(x_next, A, Q);
}
bool KalmanFilter::predict(const Eigen::MatrixXd & u) {return predict(u, A_, B_, Q_);}

bool KalmanFilter::update(
  const Eigen::MatrixXd & y, const Eigen::MatrixXd & y_pred, const Eigen::MatrixXd & C,
  const Eigen::MatrixXd & R)
{
  assert(P_.cols() == C.cols());
  assert(R.rows() == R.cols());
  assert(R.rows() == C.rows());
  assert(y.rows() == y_pred.rows());
  assert(y.rows() == C.rows());
  const Eigen::MatrixXd PCT = P_ * C.transpose();
  const Eigen::MatrixXd K = PCT * ((R + C * PCT).inverse());

  if (HasNan(K.array()) || HasInf(K.array())) {
    return false;
  }

  x_ = x_ + K * (y - y_pred);
  P_ = P_ - K * (C * P_);
  return true;
}

// x <- x + K * (y - C * x)
bool KalmanFilter::update(
  const Eigen::MatrixXd & y, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R)
{
  assert(C.cols() == x_.rows());
  const Eigen::MatrixXd y_pred = C * x_;
  return update(y, y_pred, C, R);
}
bool KalmanFilter::update(const Eigen::MatrixXd & y) {return update(y, C_, R_);}
