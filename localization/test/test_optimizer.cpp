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

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tuple>

#include "lidar_feature_localization/alignment.hpp"
#include "lidar_feature_localization/optimizer.hpp"


const Eigen::Isometry3d MakeTransform(const Eigen::Quaterniond & q, const Eigen::Vector3d & t)
{
  Eigen::Isometry3d transform;
  transform.linear() = q.toRotationMatrix();
  transform.translation() = t;
  return transform;
}

TEST(Optimizer, Alignment)
{
  const Eigen::Quaterniond q_true = Eigen::Quaterniond(1, -1, 1, -1).normalized();
  const Eigen::Vector3d t_true(-1, 3, 2);
  const Eigen::Isometry3d transform_true = MakeTransform(q_true, t_true);

  const Eigen::Matrix<double, 4, 3> X = (
    Eigen::Matrix<double, 4, 3>() <<
     4, -3, -4,
    -3, -2, -5,
    -4,  0,  2,
    -3, -3,  3).finished();

  const Eigen::MatrixXd Y = (transform_true * X.transpose()).transpose();

  {
    const AlignmentProblem problem;
    const Optimizer optimizer(problem);

    const Eigen::Isometry3d transform_pred = optimizer.Run(std::make_tuple(X, Y), transform_true);

    EXPECT_THAT(
      (transform_true.linear() - transform_pred.linear()).norm(),
      testing::Le(1e-4));
    EXPECT_THAT(
      (transform_true.translation() - transform_pred.translation()).norm(),
      testing::Le(1e-4));
  }
}
