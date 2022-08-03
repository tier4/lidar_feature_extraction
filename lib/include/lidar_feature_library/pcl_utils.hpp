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

#ifndef LIDAR_FEATURE_LIBRARY__PCL_UTILS_HPP_
#define LIDAR_FEATURE_LIBRARY__PCL_UTILS_HPP_

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <algorithm>
#include <tuple>
#include <vector>

#include <range/v3/all.hpp>


template<typename PointType>
void ThrowsIfPointCloudIsEmpty(const typename pcl::PointCloud<PointType>::Ptr & cloud)
{
  if (cloud->size() == 0) {
    throw std::invalid_argument("Point cloud is empty!");
  }
}

template<typename PointType>
Eigen::Vector3d GetXYZ(const PointType & point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

pcl::PointXYZ MakePointXYZ(const Eigen::Vector3d & v);

Eigen::MatrixXd Get(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pointcloud,
  const std::vector<int> & indices);

template<typename T>
std::vector<Eigen::Vector3d> PointsToEigen(const std::vector<T> & cloud)
{
  auto get_xyz = [](const T & p) {
      return Eigen::Vector3d(p.x, p.y, p.z);
    };
  return cloud | ranges::views::transform(get_xyz) | ranges::to_vector;
}

template<typename T>
typename pcl::PointCloud<T>::Ptr TransformPointCloud(
  const Eigen::Affine3d & transform,
  const typename pcl::PointCloud<T>::Ptr & cloud)
{
  typename pcl::PointCloud<T>::Ptr transformed(new pcl::PointCloud<T>());
  pcl::transformPointCloud(*cloud, *transformed, transform);
  return transformed;
}

#endif  // LIDAR_FEATURE_LIBRARY__PCL_UTILS_HPP_
