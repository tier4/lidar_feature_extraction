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


#ifndef EDGE_HPP_
#define EDGE_HPP_

#include <algorithm>
#include <tuple>
#include <vector>

#include "lidar_feature_library/eigen.hpp"
#include "lidar_feature_library/pcl_utils.hpp"

#include "lidar_feature_localization/filter.hpp"
#include "lidar_feature_localization/jacobian.hpp"
#include "lidar_feature_localization/kdtree.hpp"
#include "lidar_feature_localization/matrix_type.hpp"

Eigen::VectorXd Center(const Eigen::MatrixXd & X)
{
  return X.colwise().mean();
}

Eigen::MatrixXd CalcCovariance(const Eigen::MatrixXd & X)
{
  const Eigen::MatrixXd D = X.rowwise() - Center(X).transpose();
  return D.transpose() * D / X.rows();
}

Eigen::Vector3d TripletCross(
  const Eigen::Vector3d & p0,
  const Eigen::Vector3d & p1,
  const Eigen::Vector3d & p2)
{
  return (p2 - p1).cross((p0 - p1).cross(p0 - p2));
}

Eigen::Vector3d EdgeCoefficient(
  const Eigen::Vector3d p0,
  const Eigen::Vector3d & center,
  const Eigen::Vector3d & eigenvector)
{
  const Eigen::Vector3d p1 = center + 0.1 * eigenvector;
  const Eigen::Vector3d p2 = center - 0.1 * eigenvector;
  return TripletCross(p0, p1, p2);
}

std::tuple<Eigen::Vector3d, Eigen::Matrix3d> PrincipalComponents(const Eigen::Matrix3d & C)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(C);
  return {solver.eigenvalues(), solver.eigenvectors()};
}

Eigen::Matrix<double, 1, 7> MakeEdgeJacobianRow(
  const Eigen::Vector3d & coeff,
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & p)
{
  const Eigen::Matrix<double, 3, 4> drpdq = rotationlib::DRpDq(q, p);
  return (Eigen::Matrix<double, 1, 7>() << coeff.transpose() * drpdq, coeff.transpose()).finished();
}

class Edge
{
public:
  Edge(const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_map, const int n_neighbors)
  : kdtree_(KDTreeEigen(edge_map)),
    n_neighbors_(n_neighbors)
  {
  }

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd> Make(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_scan,
    const Eigen::Isometry3d & point_to_map) const
  {

    // f(dx) \approx f(0) + J * dx + dx^T * H * dx
    // dx can be obtained by solving H * dx = -J

    std::vector<Eigen::Vector3d> coeffs(edge_scan->size());
    std::vector<Vector7d> jacobian_rows(edge_scan->size());
    std::vector<bool> flags(edge_scan->size(), false);

    const Eigen::Quaterniond q(point_to_map.rotation());

    assert(Eigen::isfinite(point_to_map.translation().array()).all());
    for (unsigned int i = 0; i < edge_scan->size(); i++) {
      const Eigen::Vector3d p = GetXYZ(edge_scan->at(i));
      const Eigen::Vector3d point_on_map = point_to_map * p;
      const auto [X, squared_distances] = kdtree_.NearestKSearch(point_on_map, n_neighbors_);
      if (squared_distances.back() >= 1.0) {
        continue;
      }

      const Eigen::Matrix3d C = CalcCovariance(X);
      const auto [eigenvalues, eigenvectors] = PrincipalComponents(C);

      if (eigenvalues(2) <= 3 * eigenvalues(1)) {
        continue;
      }

      const Eigen::Vector3d principal = eigenvectors.col(2);
      const Eigen::Vector3d coeff = EdgeCoefficient(point_on_map, Center(X), principal);
      jacobian_rows[i] = MakeEdgeJacobianRow(coeff, q, p);
      flags[i] = true;
    }

    const Eigen::MatrixXd J = VectorsToEigen<7>(Filter(flags, jacobian_rows));
    const Eigen::VectorXd b = Eigen::VectorXd::Constant(J.rows(), -1.0);
    return {J, b};
  }

private:
  const KDTreeEigen kdtree_;
  const int n_neighbors_;
};

#endif  // EDGE_HPP_
