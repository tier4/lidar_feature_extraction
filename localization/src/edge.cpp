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

#include <Eigen/Eigenvalues>

#include <tuple>

#include "lidar_feature_library/algorithm.hpp"

#include "lidar_feature_localization/edge.hpp"


Eigen::VectorXd Center(const Eigen::MatrixXd & X)
{
  return X.colwise().mean();
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> CalcMeanAndCovariance(const Eigen::MatrixXd & X)
{
  const Eigen::VectorXd mean = Center(X);
  const Eigen::MatrixXd D = X.rowwise() - mean.transpose();
  const Eigen::MatrixXd covariance = D.transpose() * D / X.rows();
  return std::make_tuple(mean, covariance);
}

Eigen::Vector3d TripletCross(
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2)
{
  return (p2 - p1).cross((p0 - p1).cross(p0 - p2));
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> PrincipalComponents(const Eigen::Matrix3d & C)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
  solver.computeDirect(C);
  return {solver.eigenvalues(), solver.eigenvectors()};
}

Eigen::Matrix<double, 3, 7> MakeEdgeJacobianRow(
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2)
{
  const Eigen::Matrix<double, 3, 4> drpdq = rotationlib::DRpDq(q, p0);
  const Eigen::Matrix3d K = rotationlib::Hat(p2 - p1);
  return (Eigen::Matrix<double, 3, 7>() << K * drpdq, K).finished();
}

Eigen::Vector3d MakeEdgeResidual(
  const Eigen::Isometry3d & transform,
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2)
{
  const Eigen::Vector3d p = transform * p0;
  return (p - p1).cross(p - p2);
}

Eigen::MatrixXd GetXYZ(const Eigen::MatrixXd & matrix)
{
  const int rows = matrix.rows();
  return matrix.block(0, 0, rows, 3);
}

bool PrincipalIsReliable(const Eigen::Vector3d & eigenvalues)
{
  const Eigen::Vector3d sorted = SortThreeValues(eigenvalues);
  return sorted(2) > sorted(1) * 3.0;
}
