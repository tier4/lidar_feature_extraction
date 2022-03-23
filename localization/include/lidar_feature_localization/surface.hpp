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


#ifndef SURFACE_HPP_
#define SURFACE_HPP_

#include <tuple>
#include <vector>

#include "lidar_feature_localization/filter.hpp"
#include "lidar_feature_localization/jacobian.hpp"
#include "lidar_feature_localization/kdtree.hpp"
#include "lidar_feature_localization/pcl_utils.hpp"

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

Eigen::Vector3d EstimatePlaneCoefficients(const Eigen::MatrixXd & X)
{
  const Eigen::VectorXd g = -1.0 * Eigen::VectorXd::Ones(X.rows());
  return SolveLinear(X, g);
}

class Surface
{
public:
  Surface(const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_map, const int n_neighbors)
  : surface_map_(surface_map),
    surface_kdtree_(KDTree<pcl::PointXYZ>(surface_map)),
    n_neighbors_(n_neighbors)
  {
  }

  std::tuple<Eigen::MatrixXd, Eigen::VectorXd> Make(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface_scan,
    const Eigen::Isometry3d & point_to_map) const
  {
    std::vector<Eigen::Vector3d> coeffs(surface_scan->size());
    std::vector<double> b_vector(surface_scan->size());
    std::vector<bool> flags(surface_scan->size(), false);

    for (unsigned int i = 0; i < surface_scan->size(); i++) {
      const Eigen::Vector3d p = point_to_map * GetXYZ(surface_scan->at(i));
      const pcl::PointXYZ q = MakePointXYZ(p);
      const auto [indices, squared_distances] = surface_kdtree_.NearestKSearch(q, n_neighbors_);
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
      b_vector[i] = -(w.dot(p) + 1.0) / norm;
      flags[i] = true;
    }

    const std::vector<pcl::PointXYZ> pcl_points = Filter(flags, *surface_scan);
    const std::vector<Eigen::Vector3d> points = PointCloudToEigen(pcl_points);
    const std::vector<Eigen::Vector3d> coeffs_filtered = Filter(flags, coeffs);
    const std::vector<double> b_filtered = Filter(flags, b_vector);

    const Eigen::Quaterniond q(point_to_map.rotation());
    const Eigen::MatrixXd J = MakeJacobian(points, coeffs_filtered, q);
    const Eigen::Map<const Eigen::VectorXd> b(b_filtered.data(), b_filtered.size());
    return {J, b};
  }

private:
  const pcl::PointCloud<pcl::PointXYZ>::Ptr surface_map_;
  const KDTree<pcl::PointXYZ> surface_kdtree_;
  const int n_neighbors_;
};

#endif  // SURFACE_HPP_
