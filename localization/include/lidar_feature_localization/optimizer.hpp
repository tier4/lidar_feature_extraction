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

#ifndef OPTIMIZER_HPP_
#define OPTIMIZER_HPP_

#include "lidar_feature_localization/matrix_type.hpp"
#include "lidar_feature_localization/math.hpp"
#include "lidar_feature_localization/posevec.hpp"
#include "lidar_feature_localization/rad2deg.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>


bool CheckConvergence(const Vector6d & dx)
{
  const float dr = dx.head(3).norm();
  const float dt = (100 * dx.tail(3)).norm();
  return dr < 0.05 && dt < 0.05;
}

Eigen::VectorXd CalcUpdate(const Eigen::MatrixXd & J, const Eigen::VectorXd & b)
{
  const Eigen::MatrixXd JtJ = J.transpose() * J;
  const Eigen::VectorXd JtB = J.transpose() * b;
  return SolveLinear(JtJ, JtB);
}

template<typename ArgumentType>
class Optimizer
{
public:
  explicit Optimizer(const OptimizationProblem<ArgumentType> & problem)
  : problem_(problem)
  {
  }

  Eigen::Isometry3d Run(
    const ArgumentType & x,
    const Eigen::Isometry3d & initial_pose) const
  {
    Vector6d posevec = MakePosevec(initial_pose);
    for (int iter = 0; iter < 30; iter++) {
      const Eigen::Isometry3d pose = MakePose(posevec);
      const auto [J, b] = problem_.Make(x, pose);
      if (J.rows() == 0) {
        continue;
      }

      const Eigen::VectorXd dx = CalcUpdate(J, b);
      posevec = UpdatePoseVec(posevec, dx);

      if (CheckConvergence(dx)) {
        break;
      }
    }
    return MakePose(posevec);
  }

private:
  const OptimizationProblem<ArgumentType> problem_;
};

#endif  // OPTIMIZER_HPP_
