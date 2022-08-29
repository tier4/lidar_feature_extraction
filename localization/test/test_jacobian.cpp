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

#include <iostream>
#include "lidar_feature_localization/jacobian.hpp"

TEST(Jacobian, MakeJacobian)
{
  std::vector<Eigen::Vector3d> points;
  points.push_back(Eigen::Vector3d(1, 3, 4));
  points.push_back(Eigen::Vector3d(3, 2, 5));

  std::vector<Eigen::Vector3d> coeffs;
  coeffs.push_back(Eigen::Vector3d(0, 1, 2));
  coeffs.push_back(Eigen::Vector3d(2, 3, 4));

  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

  const Eigen::MatrixXd J = MakeJacobian(points, coeffs, q);

  const Eigen::Matrix<double, 3, 4> drpdq0 = rotationlib::DRpDq(q, points.at(0));
  EXPECT_EQ((J.row(0).head(4) - coeffs.at(0).transpose() * drpdq0).norm(), 0.);
  EXPECT_EQ((J.row(0).tail(3) - coeffs.at(0).transpose()).norm(), 0.);

  const Eigen::Matrix<double, 3, 4> drpdq1 = rotationlib::DRpDq(q, points.at(1));
  EXPECT_EQ((J.row(1).head(4) - coeffs.at(1).transpose() * drpdq1).norm(), 0.);
  EXPECT_EQ((J.row(1).tail(3) - coeffs.at(1).transpose()).norm(), 0.);
}
