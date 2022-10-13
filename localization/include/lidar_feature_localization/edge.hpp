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


#ifndef LIDAR_FEATURE_LOCALIZATION__EDGE_HPP_
#define LIDAR_FEATURE_LOCALIZATION__EDGE_HPP_

#include <algorithm>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

#include "lidar_feature_library/eigen.hpp"
#include "lidar_feature_library/pcl_utils.hpp"
#include "lidar_feature_library/random.hpp"

#include "lidar_feature_localization/degenerate.hpp"
#include "lidar_feature_localization/filter.hpp"
#include "lidar_feature_localization/jacobian.hpp"
#include "lidar_feature_localization/kdtree.hpp"
#include "lidar_feature_localization/matrix_type.hpp"
#include "lidar_feature_localization/pointcloud_to_matrix.hpp"

Eigen::VectorXd Center(const Eigen::MatrixXd & X);

std::tuple<Eigen::VectorXd, Eigen::MatrixXd> CalcMeanAndCovariance(const Eigen::MatrixXd & X);

Eigen::Vector3d TripletCross(
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2);

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> PrincipalComponents(const Eigen::Matrix3d & C);

Eigen::Matrix<double, 3, 7> MakeEdgeJacobianRow(
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2);

Eigen::Vector3d MakeEdgeResidual(
  const Eigen::Isometry3d & transform,
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2);

Eigen::MatrixXd GetXYZ(const Eigen::MatrixXd & matrix);

bool PrincipalIsReliable(const Eigen::Vector3d & eigenvalues);

template<typename PointToVector>
class Edge
{
public:
  using PointType = typename PointToVector::PointType;

  explicit Edge(const typename pcl::PointCloud<PointType>::Ptr & edge_map, const size_t n_neighbors)
  : kdtree_(MakeKDTree<PointToVector, PointType>(edge_map)), n_neighbors_(n_neighbors)
  {
  }

  std::tuple<std::vector<Eigen::MatrixXd>, std::vector<Eigen::VectorXd>> Make(
    const typename pcl::PointCloud<PointType>::Ptr & scan,
    const Eigen::Isometry3d & point_to_map) const
  {
    // f(dx) \approx f(0) + J * dx + dx^T * H * dx
    // dx can be obtained by solving H * dx = -J

    const Eigen::Quaterniond q(point_to_map.rotation());

    const size_t n = scan->size();

    std::vector<Eigen::MatrixXd> jacobians(n);
    std::vector<Eigen::VectorXd> residuals(n);

    for (size_t i = 0; i < n; i++) {
      const Eigen::VectorXd scan_point = PointToVector::Convert(scan->at(i));
      const Eigen::VectorXd query = TransformXYZ(point_to_map, scan_point);

      const auto [neighbors, _] = kdtree_->NearestKSearch(query, n_neighbors_);

      const Eigen::MatrixXd X = GetXYZ(neighbors);
      const auto [mean, covariance] = CalcMeanAndCovariance(X);

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
      const Eigen::Matrix3d eigenvectors = solver.computeDirect(covariance).eigenvectors();

      const Eigen::Vector3d principal = eigenvectors.col(2);
      const Eigen::Vector3d p0 = scan_point.head(3);
      const Eigen::Vector3d p1 = mean - principal;
      const Eigen::Vector3d p2 = mean + principal;

      jacobians[i] = MakeEdgeJacobianRow(q, p0, p1, p2);
      residuals[i] = MakeEdgeResidual(point_to_map, p0, p1, p2);
    }

    return std::make_tuple(jacobians, residuals);
  }

private:
  const std::shared_ptr<KDTreeEigen> kdtree_;
  const size_t n_neighbors_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__EDGE_HPP_
