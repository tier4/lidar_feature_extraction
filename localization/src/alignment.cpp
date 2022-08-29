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

#include "lidar_feature_localization/alignment.hpp"


std::vector<Eigen::MatrixXd> MakeJacobian(
  const Eigen::Quaterniond & q,
  const Eigen::MatrixXd & points)
{
  std::vector<Eigen::MatrixXd> jacobians;
  for (unsigned int i = 0; i < points.rows(); i++) {
    Eigen::MatrixXd J(3, 7);
    J.block<3, 4>(0, 0) = rotationlib::DRpDq(q, points.row(i));
    J.block<3, 3>(0, 4) = Eigen::Matrix3d::Identity();
    jacobians.push_back(J);
  }
  return jacobians;
}

std::vector<Eigen::VectorXd> MakeResidual(
  const Eigen::Isometry3d & point_to_map,
  const Eigen::MatrixXd & source_points,
  const Eigen::MatrixXd & target_points)
{
  assert(source_points.rows() == target_points.rows());
  assert(source_points.cols() == target_points.cols());

  std::vector<Eigen::VectorXd> residuals;
  for (unsigned int i = 0; i < source_points.rows(); i++) {
    const Eigen::Vector3d x = source_points.row(i);
    const Eigen::Vector3d y = target_points.row(i);
    const Eigen::Vector3d r = point_to_map * x - y;
    residuals.push_back(r);
  }
  return residuals;
}

std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::VectorXd>> AlignmentProblem::Make(
  const std::tuple<Eigen::MatrixXd, Eigen::MatrixXd> & points,
  const Eigen::Isometry3d & point_to_map) const
{
  const Eigen::Quaterniond q(point_to_map.rotation());
  const auto & [source_points, target_points] = points;
  const std::vector<Eigen::MatrixXd> jacobians = MakeJacobian(q, source_points);
  const std::vector<Eigen::VectorXd> residuals = MakeResidual(
    point_to_map, source_points, target_points);
  return std::make_tuple(jacobians, residuals);
}
