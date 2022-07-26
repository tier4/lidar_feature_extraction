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

#include <tuple>

#include "lidar_feature_localization/optimizer.hpp"


bool CheckConvergence(const Eigen::Quaterniond & dq, const Eigen::Vector3d & dt)
{
  return dq.vec().norm() < 1e-3 && dt.norm() < 1e-3;
}

Eigen::VectorXd CalcUpdate(const Eigen::MatrixXd & J, const Eigen::VectorXd & r)
{
  return (J.transpose() * J).ldlt().solve(-J.transpose() * r);
}

Eigen::Matrix<double, 7, 6> MakeM(const Eigen::Quaterniond & q)
{
  const Eigen::Matrix4d L = rotationlib::LeftMultiplicationMatrix(q);
  const Eigen::Matrix<double, 4, 3> Q = 0.5 * L.block<4, 3>(0, 1);

  Eigen::Matrix<double, 7, 6> M;
  M.block<4, 3>(0, 0) = Q;
  M.block<4, 3>(0, 3) = Eigen::MatrixXd::Zero(4, 3);
  M.block<3, 3>(4, 0) = Eigen::Matrix3d::Zero();
  M.block<3, 3>(4, 3) = Eigen::Matrix3d::Identity();
  return M;
}

std::tuple<Eigen::Quaterniond, Eigen::Vector3d> CalcUpdate(
  const Eigen::MatrixXd & J,
  const Eigen::VectorXd & r,
  const Eigen::Quaterniond & q)
{
  const Eigen::Matrix<double, 7, 6> M = MakeM(q);
  const Vector6d dx = CalcUpdate(J * M, r);
  const Eigen::Quaterniond dq = AngleAxisToQuaternion(dx.head(3));
  const Eigen::Vector3d dt = dx.tail(3);
  return {dq, dt};
}
