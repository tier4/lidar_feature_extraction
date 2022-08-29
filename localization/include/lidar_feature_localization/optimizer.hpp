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

#ifndef LIDAR_FEATURE_LOCALIZATION__OPTIMIZER_HPP_
#define LIDAR_FEATURE_LOCALIZATION__OPTIMIZER_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tuple>
#include <vector>

#include "lidar_feature_localization/robust.hpp"
#include "lidar_feature_localization/matrix_type.hpp"
#include "lidar_feature_localization/math.hpp"
#include "lidar_feature_localization/posevec.hpp"
#include "lidar_feature_localization/rad2deg.hpp"

#include "rotationlib/quaternion.hpp"


Eigen::VectorXd ComputeErrors(const std::vector<Eigen::VectorXd> & residuals);
std::tuple<Eigen::VectorXd, double> ComputeWeights(const Eigen::VectorXd & errors);
bool CheckConvergence(const Eigen::Quaterniond & dq, const Eigen::Vector3d & dt);

Eigen::Matrix<double, 7, 6> MakeM(const Eigen::Quaterniond & q);

std::tuple<Eigen::Quaterniond, Eigen::Vector3d> CalcUpdate(
  const Eigen::Quaterniond & q,
  const Eigen::VectorXd & weights,
  const std::vector<Eigen::MatrixXd> & jacobians,
  const std::vector<Eigen::VectorXd> & residuals);

std::tuple<Eigen::VectorXd, double> ComputeWeights(const Eigen::VectorXd & errors);

// Sola, Joan. Course on SLAM. Technical Report IRI-TR-16-04, Institut de Robòtica i, 2017.
// Section 4.2.5
template<typename ProblemType, typename ArgumentType>
class Optimizer
{
public:
  explicit Optimizer(const ProblemType & problem)
  : problem_(problem)
  {
  }

  Eigen::Isometry3d Run(
    const ArgumentType & x,
    const Eigen::Isometry3d & initial_pose) const
  {
    Eigen::Quaterniond q(initial_pose.linear());
    Eigen::Vector3d t(initial_pose.translation());

    double scale_prev;

    for (int iter = 0; iter < 10; iter++) {
      const Eigen::Isometry3d pose = MakePose(q, t);
      const auto [jacobians, residuals] = problem_.Make(x, pose);

      const Eigen::VectorXd errors = ComputeErrors(residuals);
      const auto [weights, scale] = ComputeWeights(errors);

      if (jacobians.size() == 0) {
        break;
      }

      if (iter != 0 && scale > scale_prev) {
        return MakePose(q, t);
      }

      scale_prev = scale;

      const auto [dq, dt] = CalcUpdate(q, weights, jacobians, residuals);

      q = q * dq;
      t = t + dt;

      if (CheckConvergence(dq, dt)) {
        break;
      }
    }
    return MakePose(q, t);
  }

private:
  const ProblemType problem_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__OPTIMIZER_HPP_
