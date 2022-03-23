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

#ifndef LOAM_HPP_
#define LOAM_HPP_


#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

#include <range/v3/all.hpp>

#include <algorithm>
#include <tuple>
#include <vector>

#include "lidar_feature_localization/edge.hpp"
#include "lidar_feature_localization/optimization_problem.hpp"
#include "lidar_feature_localization/surface.hpp"
#include "lidar_feature_localization/math.hpp"


using EdgeSurfaceScan = std::tuple<
  pcl::PointCloud<pcl::PointXYZ>::Ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr>;


const int n_neighbors = 5;


class LOAMOptimizationProblem
{
public:
  LOAMOptimizationProblem(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_map,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_map)
  : edge_(edge_map, n_neighbors), surface_(surface_map, n_neighbors)
  {
  }

  bool IsDegenerate(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_scan,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_scan,
    const Eigen::Isometry3d & point_to_map) const
  {
    const auto [J, b] = this->Make(std::make_tuple(edge_scan, surface_scan), point_to_map);
    const Eigen::MatrixXd JtJ = J.transpose() * J;
    return ::IsDegenerate(JtJ);
  }

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd>
  Make(const EdgeSurfaceScan & edge_surface_scan, const Eigen::Isometry3d & point_to_map) const
  {
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_scan = std::get<0>(edge_surface_scan);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_scan = std::get<1>(edge_surface_scan);
    const auto [edge_jacobian, edge_residual] = edge_.Make(edge_scan, point_to_map);
    const auto [surface_jacobian, surface_residual] = surface_.Make(surface_scan, point_to_map);

    assert(edge_jacobian.cols() == surface_jacobian.cols());

    Eigen::VectorXd b(edge_residual.size() + surface_residual.size());
    Eigen::MatrixXd J(edge_jacobian.rows() + surface_jacobian.rows(), edge_jacobian.cols());

    b << edge_residual, surface_residual;
    J << edge_jacobian, surface_jacobian;
    return {J, b};
  }

private:
  const Edge edge_;
  const Surface surface_;
};

#endif  // LOAM_HPP_
