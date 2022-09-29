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

#ifndef KALMAN_FILTER__KALMAN_FILTER_HPP_
#define KALMAN_FILTER__KALMAN_FILTER_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

/**
 * @file kalman_filter.h
 * @brief kalman filter class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

inline Eigen::VectorXd predictNextState(
  const Eigen::VectorXd & x,
  const Eigen::VectorXd & u,
  const Eigen::MatrixXd & A,
  const Eigen::MatrixXd & B)
{
  return A * x + B * u;
}

inline Eigen::MatrixXd predictNextCovariance(
  const Eigen::MatrixXd & P,
  const Eigen::MatrixXd & A,
  const Eigen::MatrixXd & Q)
{
  return A * P * A.transpose() + Q;
}

inline Eigen::MatrixXd calcKalmanGain(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & C, const Eigen::MatrixXd & R)
{
  const Eigen::MatrixXd PCT = P * C.transpose();
  return PCT * ((R + C * PCT).inverse());
}

inline Eigen::VectorXd updateState(
  const Eigen::VectorXd & x, const Eigen::VectorXd & y,
  const Eigen::MatrixXd & C, const Eigen::MatrixXd & K)
{
  return x + K * (y - C * x);
}

inline Eigen::MatrixXd updateCovariance(
  const Eigen::MatrixXd & P, const Eigen::MatrixXd & C, const Eigen::MatrixXd & K)
{
  return P - K * C * P;
}


class KalmanFilter
{
public:
  /**
   * @brief No initialization constructor.
   */
  KalmanFilter() = delete;

  /**
   * @brief constructor with initialization
   * @param x initial state
   * @param A coefficient matrix of x for process model
   * @param B coefficient matrix of u for process model
   * @param C coefficient matrix of x for measurement model
   * @param Q covariance matrix for process model
   * @param R covariance matrix for measurement model
   * @param P initial covariance of estimated state
   */
  KalmanFilter(
    const Eigen::VectorXd & x, const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & C, const Eigen::MatrixXd & Q, const Eigen::MatrixXd & R,
    const Eigen::MatrixXd & P);

  /**
   * @brief destructor
   */
  ~KalmanFilter();

  /**
   * @brief initialization of kalman filter
   * @param x initial state
   * @param P initial covariance of estimated state
   */
  void init(const Eigen::VectorXd & x, const Eigen::MatrixXd & P0);

  /**
   * @brief get component of current kalman filter state
   * @param i index of kalman filter state
   * @return value of i's component of the kalman filter state x[i]
   */
  double getXelement(unsigned int i) const;

  /**
   * @brief calculate kalman filter state and covariance by prediction model with A, B, Q matrix.
   * This is mainly for EKF with variable matrix.
   * @param u input for model
   * @param A coefficient matrix of x for process model
   * @param B coefficient matrix of u for process model
   * @param Q covariance matrix for process model
   */
  void predict(
    const Eigen::VectorXd & u, const Eigen::MatrixXd & A, const Eigen::MatrixXd & B,
    const Eigen::MatrixXd & Q);

  /**
   * @brief calculate kalman filter state by prediction model with A, B and Q being class member
   * variables.
   * @param u input for the model
   */
  void predict(const Eigen::VectorXd & u);

  /**
   * @brief calculate kalman filter state by measurement model with C and R being class member
   * variables.
   * @param y measured values
   */
  void update(const Eigen::VectorXd & y);

protected:
  Eigen::VectorXd x_;  //!< @brief current estimated state
  Eigen::MatrixXd P_;  //!< @brief covariance of estimated state
  const Eigen::MatrixXd A_;  //!< @brief process model x[k+1] = A*x[k] + B*u[k]
  const Eigen::MatrixXd B_;  //!< @brief process model x[k+1] = A*x[k] + B*u[k]
  const Eigen::MatrixXd C_;  //!< @brief measurement model y[k] = C * x[k]
  const Eigen::MatrixXd Q_;  //!< @brief covariance for process model x[k+1] = A*x[k] + B*u[k]
  const Eigen::MatrixXd R_;  //!< @brief covariance for measurement model y[k] = C * x[k]
};
#endif  // KALMAN_FILTER__KALMAN_FILTER_HPP_
