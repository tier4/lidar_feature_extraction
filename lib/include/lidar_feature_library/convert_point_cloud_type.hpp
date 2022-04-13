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

#ifndef LIDAR_FEATURE_LIBRARY__CONVERT_POINT_CLOUD_TYPE_HPP_
#define LIDAR_FEATURE_LIBRARY__CONVERT_POINT_CLOUD_TYPE_HPP_

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<typename SourcePointType, typename TargetPointType>
typename pcl::PointCloud<TargetPointType>::Ptr ConvertPointCloudType(
  const typename pcl::PointCloud<SourcePointType>::Ptr & source)
{
  typename pcl::PointCloud<TargetPointType>::Ptr target(new pcl::PointCloud<TargetPointType>());
  pcl::copyPointCloud(*source, *target);
  return target;
}

template<typename T>
pcl::PointCloud<pcl::PointXYZ>::Ptr ToPointXYZ(const typename pcl::PointCloud<T>::Ptr & cloud)
{
  return ConvertPointCloudType<T, pcl::PointXYZ>(cloud);
}

#endif  // LIDAR_FEATURE_LIBRARY__CONVERT_POINT_CLOUD_TYPE_HPP_
