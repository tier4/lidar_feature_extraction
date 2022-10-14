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

#ifndef LIDAR_FEATURE_LOCALIZATION__LOAM_OPTIMIZATION_PROBLEM_HPP_
#define LIDAR_FEATURE_LOCALIZATION__LOAM_OPTIMIZATION_PROBLEM_HPP_

#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

#include <algorithm>
#include <tuple>
#include <vector>

#include "lidar_feature_localization/edge.hpp"
#include "lidar_feature_localization/edge_surface_scan.hpp"
#include "lidar_feature_localization/degenerate.hpp"
#include "lidar_feature_localization/loam_optimization_problem.hpp"
#include "lidar_feature_localization/math.hpp"
#include "lidar_feature_localization/surface.hpp"


template<typename PointToVector>
class LOAMOptimizationProblem
{
public:
  LOAMOptimizationProblem(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_map,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_map,
    const int n_neighbors)
  : edge_(edge_map, n_neighbors), surface_(surface_map, n_neighbors)
  {
  }

  std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::VectorXd>>
  Make(const EdgeSurfaceScan & edge_surface_scan, const Eigen::Isometry3d & point_to_map) const
  {
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_scan = std::get<0>(edge_surface_scan);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_scan = std::get<1>(edge_surface_scan);

    const auto edge = edge_.Make(edge_scan, point_to_map);
    const auto surface = surface_.Make(surface_scan, point_to_map);

    const std::vector<Eigen::MatrixXd> edge_jacobian = std::get<0>(edge);
    const std::vector<Eigen::MatrixXd> surface_jacobian = std::get<0>(surface);
    const std::vector<Eigen::VectorXd> edge_residual = std::get<1>(edge);
    const std::vector<Eigen::VectorXd> surface_residual = std::get<1>(surface);

    // TODO(IshitaTakeshi) avoid memory copies
    std::vector<Eigen::MatrixXd> jacobians;
    std::vector<Eigen::VectorXd> residuals;
    jacobians.insert(jacobians.end(), edge_jacobian.begin(), edge_jacobian.end());
    jacobians.insert(jacobians.end(), surface_jacobian.begin(), surface_jacobian.end());
    residuals.insert(residuals.end(), edge_residual.begin(), edge_residual.end());
    residuals.insert(residuals.end(), surface_residual.begin(), surface_residual.end());
    return std::make_tuple(jacobians, residuals);
  }

private:
  const Edge<PointToVector> edge_;
  const Surface<PointToVector> surface_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__LOAM_OPTIMIZATION_PROBLEM_HPP_
