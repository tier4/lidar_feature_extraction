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
#include <rclcpp/rclcpp.hpp>

#include "ekf_localizer/check.hpp"


class EKFLocalizerTestSuite : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  void SetUp()
  {
    node = std::make_shared<rclcpp::Node>("node");
    warning = std::make_shared<Warning>(node.get());
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<Warning> warning;
};

TEST_F(EKFLocalizerTestSuite, CheckFrameId)
{
  EXPECT_TRUE(CheckFrameId(*warning, "map", "map"));
  EXPECT_TRUE(CheckFrameId(*warning, "base_link", "base_link"));
  EXPECT_FALSE(CheckFrameId(*warning, "map", "base_link"));
}

TEST_F(EKFLocalizerTestSuite, CheckDelayTime)
{
  EXPECT_TRUE(CheckDelayTime(*warning, 0.1));
  EXPECT_TRUE(CheckDelayTime(*warning, 0.0));
  EXPECT_FALSE(CheckDelayTime(*warning, -0.1));
}

TEST_F(EKFLocalizerTestSuite, CheckDelayStep)
{
  EXPECT_TRUE(CheckDelayStep(*warning, 9, 10));
  EXPECT_FALSE(CheckDelayStep(*warning, 10, 10));
  EXPECT_FALSE(CheckDelayStep(*warning, 11, 10));
}

TEST_F(EKFLocalizerTestSuite, CheckMeasurementMatrixNanInf)
{
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::nan("");

  EXPECT_TRUE(CheckMeasurementMatrixNanInf(*warning, Eigen::Vector3d(0., 0., 1.)));
  EXPECT_TRUE(CheckMeasurementMatrixNanInf(*warning, Eigen::Vector3d(1e16, 0., 1.)));
  EXPECT_FALSE(CheckMeasurementMatrixNanInf(*warning, Eigen::Vector3d(nan, 1., 0.)));
  EXPECT_FALSE(CheckMeasurementMatrixNanInf(*warning, Eigen::Vector3d(0., 1., inf)));
}

TEST_F(EKFLocalizerTestSuite, CheckMahalanobisGate)
{
  Eigen::Vector2d x(0, 1);
  Eigen::Vector2d y(3, 2);
  Eigen::Matrix2d C;
  C <<
    10, 0,
    0, 10;

  EXPECT_FALSE(CheckMahalanobisGate(*warning, 0.99, x, y, C));
  EXPECT_FALSE(CheckMahalanobisGate(*warning, 1.00, x, y, C));
  EXPECT_TRUE(CheckMahalanobisGate(*warning, 1.01, x, y, C));
}
