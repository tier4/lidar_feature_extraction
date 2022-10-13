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

#ifndef LIDAR_FEATURE_LOCALIZATION__LOCALIZER_HPP_
#define LIDAR_FEATURE_LOCALIZATION__LOCALIZER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tuple>

#include <rclcpp/rclcpp.hpp>

#include "lidar_feature_localization/edge.hpp"
#include "lidar_feature_localization/loam_optimization_problem.hpp"
#include "lidar_feature_localization/optimizer.hpp"

#include "lidar_feature_library/convert_point_cloud_type.hpp"


constexpr size_t N_NEIGHBORS = 15;


template<typename PointToVector, typename PointType>
class Localizer
{
  using OptimizerType = Optimizer<
    LOAMOptimizationProblem<PointToVector>,
    EdgeSurfaceScan>;

public:
  explicit Localizer(
    const typename pcl::PointCloud<PointType>::Ptr & edge_map,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_map,
    const int max_iter)
  : problem_(LOAMOptimizationProblem<PointToVector>(edge_map, surface_map, N_NEIGHBORS)),
    optimizer_(problem_, max_iter),
    is_initialized_(false),
    pose_(Eigen::Isometry3d::Identity())
  {
  }

  void Init(const Eigen::Isometry3d & initial_pose)
  {
    pose_ = initial_pose;
    is_initialized_ = true;
  }

  bool Update(const EdgeSurfaceScan & scan)
  {
    const OptimizationResult result = optimizer_.Run(scan, pose_);

    pose_ = result.pose;
    return result.success;
  }

  Eigen::Isometry3d Get() const
  {
    return pose_;
  }

  bool IsInitialized() const
  {
    return is_initialized_;
  }

private:
  const LOAMOptimizationProblem<PointToVector> problem_;
  const OptimizerType optimizer_;

  bool is_initialized_;
  Eigen::Isometry3d pose_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__LOCALIZER_HPP_
