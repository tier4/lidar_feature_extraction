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

#ifndef LIDAR_FEATURE_LOCALIZATION__KDTREE_HPP_
#define LIDAR_FEATURE_LOCALIZATION__KDTREE_HPP_

#include <pcl/point_cloud.h>

#include <memory>
#include <tuple>
#include <vector>

#include <nanoflann.hpp>

#include "lidar_feature_library/eigen.hpp"

#include "lidar_feature_localization/pointcloud_to_matrix.hpp"


constexpr bool row_major = true;
constexpr int32_t dimension = -1;

using matrix_t = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using KDTreeType = nanoflann::KDTreeEigenMatrixAdaptor<
  matrix_t,
  dimension,
  nanoflann::metric_L2,
  row_major
>;

class KDTreeEigen
{
public:
  KDTreeEigen() = delete;

  KDTreeEigen(const Eigen::MatrixXd & map, const int max_leaf_size);

  std::tuple<Eigen::MatrixXd, std::vector<double>> NearestKSearch(
    const Eigen::VectorXd & query, const int n_neighbors) const;

private:
  const Eigen::MatrixXd map_;
  const std::shared_ptr<KDTreeType> kdtree_;
};

template<typename PointToVector, typename PointType>
std::shared_ptr<KDTreeEigen> MakeKDTree(const typename pcl::PointCloud<PointType>::Ptr & map)
{
  const Eigen::MatrixXd matrix = PointCloudToMatrix<PointToVector, PointType>(map);
  return std::make_shared<KDTreeEigen>(matrix, 10);
}

#endif  // LIDAR_FEATURE_LOCALIZATION__KDTREE_HPP_
