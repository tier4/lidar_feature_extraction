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
#include <tf2/LinearMath/Quaternion.h>

#include "lidar_feature_library/eigen.hpp"


TEST(RosMsg, GetEigenCovariance)
{
  const std::array<double, 36ul> covarinace = {
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 13, 14, 15, 16, 17,
    18, 19, 20, 21, 22, 23,
    24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35
  };

  const Matrix6d C = GetEigenCovariance(covarinace);
  EXPECT_EQ(C(0, 0), 0);
  EXPECT_EQ(C(2, 0), 12);
  EXPECT_EQ(C(3, 4), 22);
  EXPECT_EQ(C(5, 2), 32);
}

TEST(RosMsg, FromEigenCovariance)
{
  Matrix6d C;
  C <<
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    12, 13, 14, 15, 16, 17,
    18, 19, 20, 21, 22, 23,
    24, 25, 26, 27, 28, 29,
    30, 31, 32, 33, 34, 35;

  const std::array<double, 36ul> covarinace = FromEigenCovariance(C);
  EXPECT_EQ(covarinace[0 * 6 + 0], 0);
  EXPECT_EQ(covarinace[2 * 6 + 0], 12);
  EXPECT_EQ(covarinace[3 * 6 + 4], 22);
  EXPECT_EQ(covarinace[5 * 6 + 2], 32);
}

TEST(Transform, MakeIsometry3d)
{
  const Eigen::Quaterniond q = Eigen::Quaterniond(-1., 1., 1., 1).normalized();
  const Eigen::Vector3d t(1., -1., 2.);
  const Eigen::Isometry3d transform = MakeIsometry3d(q, t);

  const Eigen::Vector3d p(2., 4., 1.);

  EXPECT_THAT((q * p + t - transform * p).norm(), testing::Le(1e-3));
}

TEST(TransformXYZ, TransformXYZ)
{
  Eigen::VectorXd p0(5);
  p0 << 1, 0, 2, 5, 4;

  Eigen::Isometry3d transform;
  transform.linear() <<
    -1., -0., 0.,
    0., 1., -0.,
    0., 0., -1.;
  transform.translation() <<
    2, 4, 1;

  const Eigen::VectorXd p1 = TransformXYZ(transform, p0);

  Eigen::VectorXd expected(5);
  expected << 1, 4, -1, 5, 4;

  EXPECT_THAT((p1 - expected).norm(), 0);
}

TEST(Eigen, VectorsToEigen)
{
  const std::vector<Eigen::Vector2d> vectors = {
    Eigen::Vector2d(0, 1),
    Eigen::Vector2d(2, 3),
    Eigen::Vector2d(4, 5),
    Eigen::Vector2d(6, 7)
  };

  const Eigen::MatrixXd M = VectorsToEigen<2>(vectors);

  const Eigen::MatrixXd expected =
    (Eigen::MatrixXd(4, 2) <<
    0, 1,
    2, 3,
    4, 5,
    6, 7
    ).finished();

  EXPECT_EQ(M.rows(), 4);
  EXPECT_EQ(M.cols(), 2);

  EXPECT_EQ((M - expected).norm(), 0.);
}

TEST(Eigen, HorizontalStack)
{
  const Eigen::Matrix<double, 2, 3> M0 = (
    Eigen::Matrix<double, 2, 3>() <<
      0, 1, 2,
      3, 4, 5).finished();

  const Eigen::Matrix<double, 2, 3> M1 = (
    Eigen::Matrix<double, 2, 3>() <<
      1, 3, 5,
      2, 4, 6).finished();

  const std::vector<Eigen::Matrix<double, 2, 3>> vectors{M0, M1};

  const Eigen::MatrixXd M = HorizontalStack(vectors);

  const Eigen::MatrixXd expected =
    (Eigen::MatrixXd(4, 3) <<
    0, 1, 2,
    3, 4, 5,
    1, 3, 5,
    2, 4, 6
    ).finished();

  EXPECT_EQ(M.rows(), 4);
  EXPECT_EQ(M.cols(), 3);

  EXPECT_EQ((M - expected).norm(), 0.);
}

TEST(Eigen, VectorToEigen)
{
  const std::vector<double> vector{0, 1, 2, 3};

  const Eigen::VectorXd v = VectorToEigen(vector);
  const Eigen::Vector4d expected(0, 1, 2, 3);

  EXPECT_EQ(v.size(), 4);
  EXPECT_EQ((v - expected).norm(), 0.);
}

TEST(Eigen, GetRows)
{
  const Eigen::MatrixXd matrix =
    (Eigen::MatrixXd(4, 3) <<
    0, 1, 2,
    3, 4, 5,
    1, 3, 5,
    2, 4, 6
    ).finished();

  std::vector<std::uint64_t> indices{0, 2};

  const Eigen::MatrixXd rows = GetRows(matrix, indices);
  EXPECT_EQ(rows.rows(), 2);
  EXPECT_EQ(rows.cols(), 3);
}
