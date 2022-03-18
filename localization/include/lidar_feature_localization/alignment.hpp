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

#ifndef ALGINMENT_HPP_
#define ALGINMENT_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tuple>

#include "lidar_feature_localization/math.hpp"
#include "lidar_feature_localization/optimization_problem.hpp"
#include "rotationlib/jacobian/quaternion.hpp"


Eigen::MatrixXd MakeJacobian(
  const Eigen::Quaterniond & q,
  const Eigen::MatrixXd & points)
{
  Eigen::MatrixXd J(3 * points.rows(), 7);
  for (unsigned int i = 0; i < points.rows(); i++) {
    J.block<3, 4>(3 * i, 0) = rotationlib::DRpDq(q, points.row(i));
    J.block<3, 3>(3 * i, 4) = Eigen::Matrix3d::Identity();
  }
  return J;
}

Eigen::VectorXd MakeResidual(
  const Eigen::Isometry3d & point_to_map,
  const Eigen::MatrixXd & source_points,
  const Eigen::MatrixXd & target_points)
{
  assert(source_points.rows() == target_points.rows());
  assert(source_points.cols() == target_points.cols());

  Eigen::VectorXd r(3 * source_points.rows());
  for (unsigned int i = 0; i < source_points.rows(); i++) {
    const Eigen::Vector3d x = source_points.row(i);
    const Eigen::Vector3d y = target_points.row(i);
    r.segment(3 * i, 3) = point_to_map * x - y;
  }
  return r;
}

using Points = std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>;

class AlignmentProblem : public OptimizationProblem<Points>
{
public:
  AlignmentProblem() {}

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd>
  Make(const Points & points, const Eigen::Isometry3d & point_to_map) const
  {
    const Eigen::Quaterniond q(point_to_map.rotation());
    const auto & [source_points, target_points] = points;
    const Eigen::MatrixXd J = MakeJacobian(q, source_points);
    const Eigen::VectorXd r = MakeResidual(point_to_map, source_points, target_points);
    return std::make_tuple(J, r);
  }
};

#endif  // ALGINMENT_HPP_
