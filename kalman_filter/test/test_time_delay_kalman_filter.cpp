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