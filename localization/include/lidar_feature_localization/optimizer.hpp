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

#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include "lidar_feature_localization/math.hpp"
#include "lidar_feature_localization/matrix_type.hpp"
#include "lidar_feature_localization/optimization_result.hpp"
#include "lidar_feature_localization/posevec.hpp"
#include "lidar_feature_localization/rad2deg.hpp"
#include "lidar_feature_localization/robust.hpp"

#include "rotationlib/quaternion.hpp"

bool CheckConvergence(const Eigen::Quaterniond & dq, const Eigen::Vector3d & dt);

Vector6d WeightedUpdate(
  const Eigen::Matrix<double, 7, 6> & M,
  const Eigen::VectorXd & weights,
  const std::vector<Eigen::MatrixXd> & jacobians,
  const std::vector<Eigen::VectorXd> & residuals);

Eigen::Matrix<double, 7, 6> MakeM(const Eigen::Quaterniond & q);

std::tuple<Eigen::Quaterniond, Eigen::Vector3d> CalcUpdate(
  const Eigen::Quaterniond & q,
  const Eigen::VectorXd & weights,
  const std::vector<Eigen::MatrixXd> & jacobians,
  const std::vector<Eigen::VectorXd> & residuals);

Eigen::VectorXd ComputeErrors(const std::vector<Eigen::VectorXd> & residuals);
std::tuple<Eigen::VectorXd, double> NormalizeErrorScale(const Eigen::VectorXd & errors);
Eigen::VectorXd ComputeWeights(const Eigen::VectorXd & scale_normalized_errors);

// Sola, Joan. Course on SLAM. Technical Report IRI-TR-16-04, Institut de Robòtica i, 2017.
// Section 4.2.5
template<typename ProblemType, typename ArgumentType>
class Optimizer
{
public:
  explicit Optimizer(const ProblemType & problem, const int max_iter = 20)
  : problem_(problem), max_iter_(max_iter)
  {
  }

  OptimizationResult Run(
    const ArgumentType & x,
    const Eigen::Isometry3d & initial_pose) const
  {
    Eigen::Quaterniond q(initial_pose.linear());
    Eigen::Vector3d t(initial_pose.translation());

    double prev_scale = std::numeric_limits<double>::max();
    double prev_error = std::numeric_limits<double>::max();

    for (int iter = 0; iter < max_iter_; iter++) {
      const Eigen::Isometry3d pose = MakePose(q, t);
      const auto [jacobians, residuals] = problem_.Make(x, pose);

      if (residuals.size() == 0) {
        return EmptyInput(MakePose(q, t), iter);
      }

      const Eigen::VectorXd errors = ComputeErrors(residuals);
      const auto [normalized, scale] = NormalizeErrorScale(errors);
      const double error = errors.sum();

      if (error > prev_error) {
        return LargerErrorThanPrevious(MakePose(q, t), iter, error, scale);
      }
      prev_error = error;

      if (scale > prev_scale) {
        return LargerScaleThanPrevious(MakePose(q, t), iter, error, scale);
      }
      prev_scale = scale;

      const Eigen::VectorXd weights = ComputeWeights(normalized);
      const auto [dq, dt] = CalcUpdate(q, weights, jacobians, residuals);

      q = q * dq;
      t = t + dt;

      if (CheckConvergence(dq, dt)) {
        return SuccessfullyConverged(MakePose(q, t), iter, error, scale);
      }
    }

    return ReachedMaximumIteration(MakePose(q, t), max_iter_, prev_error, prev_scale);
  }

private:
  const ProblemType problem_;
  const int max_iter_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__OPTIMIZER_HPP_
