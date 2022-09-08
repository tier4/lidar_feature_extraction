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

#ifndef LIDAR_FEATURE_LOCALIZATION__OPTIMIZATION_RESULT_HPP_
#define LIDAR_FEATURE_LOCALIZATION__OPTIMIZATION_RESULT_HPP_

#include <Eigen/Geometry>

#include <string>


struct OptimizationResult
{
  const Eigen::Isometry3d pose;
  const std::string message;
  const int iteration;
  const double error;
  const double error_scale;
  const bool success;
};

inline OptimizationResult EmptyInput(const Eigen::Isometry3d & pose, const int iter)
{
  const std::string message = "The input data is empty";
  return OptimizationResult{pose, message, iter, 0., 0., false};
}

inline OptimizationResult LargerErrorThanPrevious(
  const Eigen::Isometry3d & pose, const int iter, const double error, const double scale)
{
  const std::string message = "The error is larger than previous iteration";
  return OptimizationResult{pose, message, iter, error, scale, true};
}

inline OptimizationResult LargerScaleThanPrevious(
  const Eigen::Isometry3d & pose, const int iter, const double error, const double scale)
{
  const std::string message = "The scale is larger than previous iteration";
  return OptimizationResult{pose, message, iter, error, scale, true};
}

inline OptimizationResult SuccessfullyConverged(
  const Eigen::Isometry3d & pose, const int iter, const double error, const double scale)
{
  const std::string message = "Optimization successfully converged";
  return OptimizationResult{pose, message, iter, error, scale, true};
}

inline OptimizationResult ReachedMaximumIteration(
  const Eigen::Isometry3d & pose, const int iter, const double error, const double scale)
{
  const std::string message = "The iteration reached the maximum value";
  return OptimizationResult{pose, message, iter, error, scale, false};
}

#endif  // LIDAR_FEATURE_LOCALIZATION__OPTIMIZATION_RESULT_HPP_
