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

#ifndef OPTIMIZATION_PROBLEM_
#define OPTIMIZATION_PROBLEM_

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

bool IsDegenerate(const Eigen::MatrixXd & C, const double threshold = 0.1)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(C);
  const Eigen::VectorXd eigenvalues = es.eigenvalues();
  return (eigenvalues.array().abs() < threshold).any();
}

template<typename ArgumentType>
class OptimizationProblem
{
public:
  std::tuple<Eigen::MatrixXd, Eigen::VectorXd>
  virtual Make(const ArgumentType &, const Eigen::Isometry3d &) const
  {
    return std::make_tuple(Eigen::MatrixXd::Zero(0, 0), Eigen::VectorXd::Zero(0, 0));
  }
};

#endif  // OPTIMIZATION_PROBLEM_
