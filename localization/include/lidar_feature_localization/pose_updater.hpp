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

#ifndef LIDAR_FEATURE_LOCALIZATION__POSE_UPDATER_HPP_
#define LIDAR_FEATURE_LOCALIZATION__POSE_UPDATER_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tuple>

#include "lidar_feature_localization/edge.hpp"
#include "lidar_feature_localization/loam_optimization_problem.hpp"
#include "lidar_feature_localization/optimizer.hpp"

const size_t N_NEIGHBORS = 15;

template<typename PointToVector>
class LOAMPoseUpdater
{
public:
  using Problem = Edge<PointToVector>;
  using PointType = typename PointToVector::PointType;
  using PointCloudPtr = typename pcl::PointCloud<PointType>::Ptr;
  using LOAMOptimizer = Optimizer<LOAMOptimizationProblem<PointToVector>, EdgeSurfaceScan>;

  explicit LOAMPoseUpdater(const PointCloudPtr & local_map)
  : optimizer_(MakeLOAMOptimizer(local_map))
  {
  }

  Eigen::Isometry3d operator()(const PointCloudPtr & scan, const Eigen::Isometry3d & pose) const
  {
    const OptimizationResult result = optimizer_.Run(scan, pose);
    return result.pose;
  }

private:
  LOAMOptimizer MakeLOAMOptimizer(const PointCloudPtr & map)
  {
    const Problem problem(map, N_NEIGHBORS);
    return LOAMOptimizer(problem);
  }

  const LOAMOptimizer optimizer_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__POSE_UPDATER_HPP_
