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

#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>

#include <range/v3/all.hpp>

#include <algorithm>
#include <tuple>
#include <vector>

#include "lidar_feature_localization/kdtree.hpp"
#include "lidar_feature_localization/jacobian.hpp"
#include "lidar_feature_localization/math.hpp"


const int n_neighbors = 5;


Eigen::Vector3d GetXYZ(const pcl::PointXYZ & point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

pcl::PointXYZ MakePointXYZ(const Eigen::Vector3d & v)
{
  return pcl::PointXYZ(v(0), v(1), v(2));
}

Eigen::MatrixXd Get(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pointcloud,
  const std::vector<int> & indices)
{
  Eigen::MatrixXd A(indices.size(), 3);
  for (const auto & [j, index] : ranges::views::enumerate(indices)) {
    A.row(j) = GetXYZ(pointcloud->at(index)).transpose();
  }
  return A;
}

Eigen::Matrix3d CalcCovariance(const Eigen::MatrixXd & X)
{
  const Eigen::Vector3d c = X.colwise().mean();
  const Eigen::MatrixXd D = X.rowwise() - c.transpose();
  return D.transpose() * D / X.rows();
}

std::vector<int> TrueIndices(const std::vector<bool> & flags)
{
  return ranges::views::iota(0, static_cast<int>(flags.size())) |
         ranges::views::filter([&](int i) {return flags[i];}) |
         ranges::to_vector;
}

template<typename Iter>
auto Filter(
  const std::vector<bool> & flags,
  const Iter & values)
{
  typedef typename Iter::value_type T;

  assert(flags.size() == values.size());
  std::vector<T> filtered;
  for (unsigned int i = 0; i < flags.size(); i++) {
    if (flags[i]) {
      filtered.push_back(values[i]);
    }
  }
  return filtered;
}

double PointPlaneDistance(const Eigen::Vector3d & w, const Eigen::Vector3d & x)
{
  return std::abs(w.dot(x) + 1.0) / w.norm();
}

bool ValidatePlane(const Eigen::MatrixXd & X, const Eigen::Vector3d & w)
{
  for (int j = 0; j < X.rows(); j++) {
    const Eigen::Vector3d x = X.row(j);
    if (PointPlaneDistance(w, x) > 0.2) {
      return false;
    }
  }
  return true;
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

template<int N>
Eigen::Vector3d Center(const Eigen::Matrix<double, N, 3> & neighbors)
{
  return neighbors.colwise().mean();
}

std::vector<Eigen::Vector3d> PointCloudToEigen(const std::vector<pcl::PointXYZ> & cloud)
{
  return cloud | ranges::views::transform(GetXYZ) | ranges::to_vector;
}

Eigen::Vector3d EstimatePlaneCoefficients(const Eigen::MatrixXd & X)
{
  const Eigen::VectorXd g = -1.0 * Eigen::VectorXd::Ones(X.rows());
  return SolveLinear(X, g);
}

class OptimizationProblem
{
public:
  OptimizationProblem(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_map,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_map)
  : edge_map_(edge_map),
    surface_map_(surface_map),
    edge_kdtree_(KDTree<pcl::PointXYZ>(edge_map)),
    surface_kdtree_(KDTree<pcl::PointXYZ>(surface_map))
  {
  }

  bool IsDegenerate(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_scan,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_scan,
    const Eigen::Isometry3d & point_to_map) const
  {
    const auto [J, b] = this->Make(edge_scan, surface_scan, point_to_map);
    const Eigen::MatrixXd JtJ = J.transpose() * J;
    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(JtJ);
    const Eigen::VectorXd eigenvalues = es.eigenvalues();
    return (eigenvalues.array() < 100.0).any();
  }

  std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>, std::vector<double>>
  FromEdge(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_scan,
    const Eigen::Isometry3d & point_to_map) const
  {
    // f(dx) \approx f(0) + J * dx + dx^T * H * dx
    // dx can be obtained by solving H * dx = -J

    std::vector<Eigen::Vector3d> coeffs(edge_scan->size());
    std::vector<bool> flags(edge_scan->size(), false);

    for (unsigned int i = 0; i < edge_scan->size(); i++) {
      const Eigen::Vector3d p0 = point_to_map * GetXYZ(edge_scan->at(i));
      const pcl::PointXYZ q = MakePointXYZ(p0);
      const auto [indices, squared_distances] = edge_kdtree_.nearestKSearch(q, n_neighbors);
      if (squared_distances.back() >= 1.0) {
        continue;
      }

      const Eigen::Matrix<double, n_neighbors, 3> neighbors = Get(edge_map_, indices);
      const Eigen::Matrix3d C = CalcCovariance(neighbors);
      const auto [eigenvalues, eigenvectors] = PrincipalComponents(C);

      if (eigenvalues(2) <= 3 * eigenvalues(1)) {
        continue;
      }

      const Eigen::Vector3d principal = eigenvectors.col(2);
      coeffs[i] = EdgeCoefficient(p0, Center(neighbors), principal);
      flags[i] = true;
    }

    const std::vector<pcl::PointXYZ> pcl_points = Filter(flags, *edge_scan);
    const std::vector<Eigen::Vector3d> points = PointCloudToEigen(pcl_points);
    const std::vector<Eigen::Vector3d> coeffs_filtered = Filter(flags, coeffs);
    const std::vector<double> b(coeffs_filtered.size(), -1.0);
    return {points, coeffs_filtered, b};
  }

  std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>, std::vector<double>>
  FromSurface(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_scan,
    const Eigen::Isometry3d & point_to_map) const
  {
    std::vector<Eigen::Vector3d> coeffs(surface_scan->size());
    std::vector<double> b(surface_scan->size());
    std::vector<bool> flags(surface_scan->size(), false);

    for (unsigned int i = 0; i < surface_scan->size(); i++) {
      const Eigen::Vector3d p = point_to_map * GetXYZ(surface_scan->at(i));
      const pcl::PointXYZ q = MakePointXYZ(p);
      const auto [indices, squared_distances] = surface_kdtree_.nearestKSearch(q, n_neighbors);
      if (squared_distances.back() >= 1.0) {
        continue;
      }

      const Eigen::MatrixXd X = Get(surface_map_, indices);
      const Eigen::Vector3d w = EstimatePlaneCoefficients(X);

      if (!ValidatePlane(X, w)) {
        continue;
      }

      const double norm = w.norm();

      coeffs[i] = w / norm;
      b[i] = -(w.dot(p) + 1.0) / norm;
      flags[i] = true;
    }

    const std::vector<pcl::PointXYZ> pcl_points = Filter(flags, *surface_scan);
    const std::vector<Eigen::Vector3d> points = PointCloudToEigen(pcl_points);
    const std::vector<Eigen::Vector3d> coeffs_filtered = Filter(flags, coeffs);
    const std::vector<double> b_filtered = Filter(flags, b);
    return {points, coeffs_filtered, b_filtered};
  }

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd>
  Make(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge_scan,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_scan,
    const Eigen::Isometry3d & point_to_map) const
  {
    const auto [edge_points, edge_coeffs, edge_b] = FromEdge(edge_scan, point_to_map);
    const auto [surface_points, surface_coeffs, surface_b] = FromSurface(surface_scan, point_to_map);

    const auto points = ranges::views::concat(edge_points, surface_points) | ranges::to_vector;
    const auto coeffs = ranges::views::concat(edge_coeffs, surface_coeffs) | ranges::to_vector;
    auto b_vector = ranges::views::concat(edge_b, surface_b) | ranges::to_vector;

    assert(points.size() == coeffs.size());
    assert(points.size() == b_vector.size());
    const Eigen::Quaterniond q(point_to_map.rotation());
    const Eigen::MatrixXd J = MakeJacobian(points, coeffs, q);
    const Eigen::Map<Eigen::VectorXd> b(b_vector.data(), b_vector.size());
    return {J, b};
  }

private:
  const pcl::PointCloud<pcl::PointXYZ>::Ptr edge_map_;
  const pcl::PointCloud<pcl::PointXYZ>::Ptr surface_map_;
  const KDTree<pcl::PointXYZ> edge_kdtree_;
  const KDTree<pcl::PointXYZ> surface_kdtree_;
};
