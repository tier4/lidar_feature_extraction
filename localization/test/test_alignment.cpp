// Copyright 2022 Tixiao Shan, Takeshi Ishita
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Tixiao Shan, Takeshi Ishita nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gmock/gmock.h>

#include "lidar_feature_localization/alignment.hpp"


TEST(Alignment, MakeJacobian)
{
  Eigen::Matrix<double, 2, 3> points;
  points.row(0) = Eigen::Vector3d(0, 1, 2);
  points.row(1) = Eigen::Vector3d(3, 4, 5);

  const Eigen::Quaterniond q = Eigen::Quaterniond(1, -1, 1, -1).normalized();

  const Eigen::MatrixXd J = MakeJacobian(q, points);
  ASSERT_EQ(J.rows(), 6);
  ASSERT_EQ(J.cols(), 7);

  {
    const Eigen::Matrix<double, 3, 4> J0 = J.block<3, 4>(0, 0);
    const Eigen::Matrix<double, 3, 4> J1 = J.block<3, 4>(3, 0);

    const Eigen::Matrix<double, 3, 4> expected0 = rotationlib::DRpDq(q, points.row(0));
    const Eigen::Matrix<double, 3, 4> expected1 = rotationlib::DRpDq(q, points.row(1));

    EXPECT_THAT((J0 - expected0).norm(), testing::Le(1e-6));
    EXPECT_THAT((J1 - expected1).norm(), testing::Le(1e-6));
  }

  {
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

    const Eigen::Matrix3d J0 = J.block<3, 3>(0, 4);
    const Eigen::Matrix3d J1 = J.block<3, 3>(3, 4);

    EXPECT_THAT((J0 - identity).norm(), testing::Le(1e-6));
    EXPECT_THAT((J1 - identity).norm(), testing::Le(1e-6));
  }
}

TEST(Alignment, MakeResidual)
{
  const Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::Vector4d::Random()).normalized();
  const Eigen::Vector3d t = Eigen::Vector3d::Random();

  Eigen::Isometry3d transform;
  transform.linear() = q.toRotationMatrix();
  transform.translation() = t;

  const int N = 2;
  const Eigen::MatrixXd source = Eigen::MatrixXd::Random(N, 3);
  const Eigen::MatrixXd target = Eigen::MatrixXd::Random(N, 3);

  const Eigen::VectorXd residual = MakeResidual(transform, source, target);

  ASSERT_EQ(residual.size(), 3 * N);
  const Eigen::Vector3d x0 = source.row(0);
  const Eigen::Vector3d y0 = target.row(0);
  const Eigen::Vector3d r0 = residual.segment(0, 3);
  EXPECT_THAT((q * x0 + t - y0 - r0).norm(), testing::Le(1e-4));

  const Eigen::Vector3d x1 = source.row(1);
  const Eigen::Vector3d y1 = target.row(1);
  const Eigen::Vector3d r1 = residual.segment(3, 3);
  EXPECT_THAT((q * x1 + t - y1 - r1).norm(), testing::Le(1e-4));
}
