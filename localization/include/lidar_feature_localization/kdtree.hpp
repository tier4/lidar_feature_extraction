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

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <tuple>
#include <vector>

#include "lidar_feature_library/pcl_utils.hpp"

template<typename PointType>
class KDTree
{
public:
  explicit KDTree(const typename pcl::PointCloud<PointType>::Ptr & points)
  {
    kdtree_.setInputCloud(points);
  }

  std::tuple<std::vector<int>, std::vector<float>> RadiusSearch(
    const PointType & point, const double radius) const
  {
    std::vector<int> indices;
    std::vector<float> squared_distances;
    kdtree_.radiusSearch(point, radius, indices, squared_distances);
    return {indices, squared_distances};
  }

  std::tuple<std::vector<int>, std::vector<float>> NearestKSearch(
    const PointType & point, const int k) const
  {
    std::vector<int> indices;
    std::vector<float> squared_distances;
    kdtree_.nearestKSearch(point, k, indices, squared_distances);
    return {indices, squared_distances};
  }

private:
  pcl::KdTreeFLANN<PointType> kdtree_;
};

template<typename PointType>
class KDTreeEigen
{
public:
  explicit KDTreeEigen(const typename pcl::PointCloud<PointType>::Ptr & map)
  : map_(map), kdtree_(KDTree<PointType>(map_))
  {
  }

  std::tuple<Eigen::MatrixXd, std::vector<float>> RadiusSearch(
    const Eigen::VectorXd & point, const double radius) const
  {
    const pcl::PointXYZ q = MakePointXYZ(point);
    const auto [indices, squared_distances] = kdtree_.RadiusSearch(q, radius);
    const Eigen::MatrixXd X = Get(map_, indices);
    return std::make_tuple(X, squared_distances);
  }

  std::tuple<Eigen::MatrixXd, std::vector<float>> NearestKSearch(
    const Eigen::VectorXd & point, const int n_neighbors) const
  {
    const pcl::PointXYZ q = MakePointXYZ(point);
    const auto [indices, squared_distances] = kdtree_.NearestKSearch(q, n_neighbors);
    const Eigen::MatrixXd X = Get(map_, indices);
    return std::make_tuple(X, squared_distances);
  }

private:
  const typename pcl::PointCloud<PointType>::Ptr map_;
  const KDTree<PointType> kdtree_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__KDTREE_HPP_
