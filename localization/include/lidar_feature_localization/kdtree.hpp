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

#include <nanoflann.hpp>
#include <pcl/point_cloud.h>

#include <memory>
#include <tuple>
#include <vector>

#include "lidar_feature_library/eigen.hpp"

using matrix_t = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using KDTreeType = nanoflann::KDTreeEigenMatrixAdaptor<matrix_t>;

class KDTreeEigen
{
public:
  explicit KDTreeEigen(const Eigen::MatrixXd & map, int max_leaf_size)
  : map_(map),
    kdtree_(std::make_shared<KDTreeType>(map.cols(), std::cref(map), max_leaf_size))
  {
  }

  std::tuple<Eigen::MatrixXd, std::vector<double>> NearestKSearch(
    const Eigen::VectorXd & query, const int n_neighbors) const
  {
    assert(map_.cols() == query.size());

    std::vector<std::uint64_t> indices(n_neighbors);
    std::vector<double> distances(n_neighbors);

    nanoflann::KNNResultSet<double> result(n_neighbors);

    result.init(&indices[0], &distances[0]);
    std::vector<double> queryvec(query.size());
    Eigen::VectorXd::Map(&queryvec[0], query.size()) = query;
    kdtree_->index->findNeighbors(result, &queryvec[0], nanoflann::SearchParams(10));
    // kdtree_->query(&query[0], n_neighbors, &indices[0], &distances[0]);

    indices.resize(n_neighbors);
    distances.resize(n_neighbors);

    const Eigen::MatrixXd X = GetRows(map_, indices);
    return std::make_tuple(X, distances);
  }

private:
  const Eigen::MatrixXd map_;
  const std::shared_ptr<KDTreeType> kdtree_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__KDTREE_HPP_
