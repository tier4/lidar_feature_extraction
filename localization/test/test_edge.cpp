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

#include "lidar_feature_localization/loam.hpp"

using testing::ElementsAre;
using testing::DoubleEq;

TEST(Edge, TripletCross)
{
  {
    const Eigen::Vector3d p0(1., 2., 3.);
    const Eigen::Vector3d p1(4., 5., 6.);
    const Eigen::Vector3d p2(7., 8., 9.);
    const Eigen::Vector3d c = TripletCross(p0, p1, p2);
    const std::vector<double> v(c.data(), c.data() + c.size());
    EXPECT_THAT(v, ElementsAre(DoubleEq(0.), DoubleEq(0.), DoubleEq(0.)));
  }

  {
    const Eigen::Vector3d p0(0., 2., 1.);
    const Eigen::Vector3d p1(2., 0., 1.);
    const Eigen::Vector3d p2(1., 3., 0.);
    const Eigen::Vector3d c = TripletCross(p0, p1, p2);
    const std::vector<double> v(c.data(), c.data() + c.size());
    EXPECT_THAT(v, ElementsAre(DoubleEq(14.), DoubleEq(2.), DoubleEq(-8.)));
  }
}

TEST(Edge, PrincipalComponents)
{
  {
    const Eigen::Matrix3d A = Eigen::Matrix3d::Random();
    const Eigen::Matrix3d C = A * A.transpose();
    const auto [eigenvalues, eigenvectors] = PrincipalComponents(C);

    EXPECT_THAT(eigenvalues(0), testing::Le(eigenvalues(1)));
    EXPECT_THAT(eigenvalues(1), testing::Le(eigenvalues(2)));

    const Eigen::Matrix3d D = eigenvalues.asDiagonal();
    const Eigen::Matrix3d V = eigenvectors;

    EXPECT_THAT((C - V * D * V.inverse()).norm(), testing::Le(1e-4));

    const double lambda = eigenvalues(2);
    const Eigen::Vector3d u = eigenvectors.col(2);
    EXPECT_THAT((C * u - lambda * u).norm(), testing::Le(1e-4));
  }

  {
    Eigen::MatrixXd X(10, 3);
    for (int i = 0; i < X.rows(); i++) {
      const double x = 0.1 * i;
      X.row(i) = Eigen::Vector3d(x, 0., 0.);
    }
    const Eigen::Matrix3d C = CalcCovariance(X);
    const auto [eigenvalues, eigenvectors] = PrincipalComponents(C);
    EXPECT_EQ((eigenvectors.col(2) - Eigen::Vector3d(1, 0, 0)).norm(), 0.);
    EXPECT_EQ(eigenvalues(0), 0.);
    EXPECT_EQ(eigenvalues(1), 0.);
    EXPECT_THAT(eigenvalues(2), testing::Gt(0.));
  }
}

TEST(Edge, Center)
{
  Eigen::Matrix<double, 5, 3> A;
  A <<
      4, 5, 1,
      2, 0, 4,
      6, 2, 2,
      7, 5, 9,
      0, 7, 7;
  const Eigen::Vector3d c = Center(A);
  std::vector<double> v(c.data(), c.data() + c.size());
  EXPECT_THAT(v, ElementsAre(3.8, 3.8, 4.6));
}

TEST(Edge, CalcCovariance)
{
  const Eigen::MatrixXd X =
    (Eigen::MatrixXd(4, 3) <<
      2, 8, 9,
      3, 5, 0,
      6, 5, 5,
      5, 2, 2).finished();

  const Eigen::Matrix3d expected =
    (Eigen::Matrix3d() <<
      10., -9., -6.,
      -9., 18., 21.,
      -6., 21., 46.).finished() / 4.;

  const Eigen::MatrixXd C = CalcCovariance(X);
  ASSERT_EQ(C.rows(), 3);
  ASSERT_EQ(C.cols(), 3);
  EXPECT_EQ((C - expected).norm(), 0.);
}
