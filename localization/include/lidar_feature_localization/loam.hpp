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

#ifndef LIDAR_FEATURE_LOCALIZATION__LOAM_HPP_
#define LIDAR_FEATURE_LOCALIZATION__LOAM_HPP_


#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

#include <range/v3/all.hpp>

#include <algorithm>
#include <tuple>
#include <vector>

#include "lidar_feature_localization/edge.hpp"
#include "lidar_feature_localization/degenerate.hpp"
#include "lidar_feature_localization/math.hpp"


const int n_neighbors = 5;


template<typename PointToVector, typename PointType>
class LOAMOptimizationProblem
{
public:
  LOAMOptimizationProblem(
    const typename pcl::PointCloud<PointType>::Ptr & edge_map)
  : edge_(edge_map, n_neighbors)
  {
  }

  bool IsDegenerate(
    const typename pcl::PointCloud<PointType>::Ptr & edge_scan,
    const Eigen::Isometry3d & point_to_map) const
  {
    const auto [J, r] = this->Make(edge_scan, point_to_map);
    const Eigen::MatrixXd JtJ = J.transpose() * J;

    return ::IsDegenerate(JtJ);
  }

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd>
  Make(
    const typename pcl::PointCloud<PointType>::Ptr & edge_scan,
    const Eigen::Isometry3d & point_to_map) const
  {
    return edge_.Make(edge_scan, point_to_map);
  }

private:
  const Edge<PointToVector, PointType> edge_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__LOAM_HPP_
