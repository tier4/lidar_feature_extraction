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

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "kalman_filter/time_delay_kalman_filter.hpp"


TEST(InitX, SmokeTest)
{
  {
    const Eigen::Vector3d x0(1, 2, 3);
    const Eigen::MatrixXd x = initX(x0, 2);

    Eigen::VectorXd expected(6);
    expected << 1, 2, 3, 1, 2, 3;

    EXPECT_EQ(x.size(), 6);
    EXPECT_EQ((x - expected).norm(), 0);
  }

  {
    const Eigen::Vector2d x0(1, 2);
    const Eigen::VectorXd x = initX(x0, 4);

    Eigen::VectorXd expected(8);
    expected << 1, 2, 1, 2, 1, 2, 1, 2;

    EXPECT_EQ(x.size(), 8);
    EXPECT_EQ((x - expected).norm(), 0);
  }
}

TEST(InitP, SmokeTest)
{
  Eigen::Matrix2d P0;
  P0 <<
    1, 2,
    3, 4;

  Eigen::MatrixXd expected(6, 6);
  expected <<
    1, 2, 0, 0, 0, 0,
    3, 4, 0, 0, 0, 0,
    0, 0, 1, 2, 0, 0,
    0, 0, 3, 4, 0, 0,
    0, 0, 0, 0, 1, 2,
    0, 0, 0, 0, 3, 4;

  const Eigen::MatrixXd P = initP(P0, 3);
  EXPECT_EQ(P.rows(), 6);
  EXPECT_EQ(P.cols(), 6);
  EXPECT_EQ((P - expected).norm(), 0);
}

TEST(UpdateX, SmokeTest)
{
  Eigen::VectorXd x = (Eigen::VectorXd(5) << 1, 2, 3, 4, 5).finished();
  Eigen::Vector2d y(10, 20);
  const Eigen::VectorXd z = updateX(x, y);
  EXPECT_EQ(z.size(), 5);

  Eigen::VectorXd expected = (Eigen::VectorXd(5) << 10, 20, 1, 2, 3).finished();
  EXPECT_EQ((z - expected).norm(), 0);
}

TEST(GetLatestX, SmokeTest)
{
  const Eigen::Vector2d x0(2, 3);
  TimeDelayKalmanFilter kf(x0, Eigen::Matrix2d::Identity(), 10);

  const Eigen::Vector2d x1(4, 5);
  kf.predictWithDelay(x1, Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Identity());

  EXPECT_EQ((kf.getLatestX() - x1).norm(), 0);

  const Eigen::Vector2d x2(6, 7);
  kf.predictWithDelay(x2, Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Identity());
  EXPECT_EQ((kf.getX(0) - x2).norm(), 0);
  EXPECT_EQ((kf.getX(1) - x1).norm(), 0);
  EXPECT_EQ((kf.getX(2) - x0).norm(), 0);
}

TEST(UpdateP, SmokeTest)
{
  Eigen::Matrix<double, 5, 5> P;
  P <<
    1, 2, 3, 4, 5,
    3, 5, 7, 9, 1,
    2, 4, 6, 8, 0,
    6, 2, 8, 4, 0,
    7, 4, 1, 8, 5;

  Eigen::Matrix<double, 2, 2> BB;
  BB <<
    1, 2,
    3, 5;

  Eigen::Matrix<double, 2, 3> BC;
  BC <<
    1, 2, 3,
    3, 5, 7;

  Eigen::Matrix<double, 3, 2> CB;
  CB <<
    1, 2,
    3, 5,
    2, 4;

  Eigen::Matrix<double, 3, 3> CC;
  CC <<
    1, 2, 3,
    3, 5, 7,
    2, 4, 6;

  Eigen::Matrix<double, 2, 2> A;
  A <<
    5, 9,
    8, 7;

  Eigen::Matrix<double, 2, 2> Q;
  Q <<
    4, 1,
    5, 2;

  const Eigen::MatrixXd updated = updateP(P, A, Q);

  EXPECT_EQ((updated.block<2, 2>(0, 0) - (A * BB * A.transpose() + Q)).norm(), 0);
  EXPECT_EQ((updated.block<2, 3>(0, 2) - (A * BC)).norm(), 0);
  EXPECT_EQ((updated.block<3, 2>(2, 0) - (CB * A.transpose())).norm(), 0);
  EXPECT_EQ((updated.block<3, 3>(2, 2) - CC).norm(), 0);
}

TEST(UpdateWithDelay, ThrowsInvalidArgumentIfDelayStepExceedsMax)
{
  const Eigen::Vector2d x = Eigen::Vector2d::Zero();
  const Eigen::Matrix2d P = Eigen::Matrix2d::Identity();
  const int max_delay_step = 10;

  TimeDelayKalmanFilter kf(x, P, max_delay_step);

  const Eigen::Vector2d y = Eigen::Vector2d::Zero();
  const Eigen::Matrix2d C = Eigen::Matrix2d::Identity();
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity();

  kf.updateWithDelay(y, C, R, max_delay_step - 1);

  EXPECT_THROW(
    try {
    kf.updateWithDelay(y, C, R, max_delay_step);
  } catch (std::invalid_argument & e) {
    EXPECT_STREQ(e.what(), "The delay step is larger than the maximum allowed value");
    throw e;
  }
    ,
    std::invalid_argument
  );
}

TEST(UpdateWithDelay, ThrowsInvalidArgumentIfKalmanGainContainsNanOrInf)
{
  const Eigen::Vector2d x = Eigen::Vector2d::Zero();
  const Eigen::Matrix2d P = Eigen::Matrix2d::Identity();
  const int max_delay_step = 10;

  TimeDelayKalmanFilter kf(x, P, max_delay_step);

  const Eigen::Vector2d y = Eigen::Vector2d::Zero();

  kf.updateWithDelay(y, Eigen::Matrix2d::Identity(), Eigen::Matrix2d::Identity(), 0);

  EXPECT_THROW(
    try {
    kf.updateWithDelay(y, Eigen::Matrix2d::Zero(), Eigen::Matrix2d::Zero(), 0);
  } catch (std::invalid_argument & e) {
    EXPECT_STREQ(e.what(), "The kalman gain contains nan or inf");
    throw e;
  }
    ,
    std::invalid_argument
  );
}
