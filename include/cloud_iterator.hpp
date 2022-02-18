// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef CLOUD_ITERATOR_HPP_
#define CLOUD_ITERATOR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

template<typename PointT>
using CloudIterator = typename pcl::PointCloud<PointT>::iterator;

template<typename PointT>
using CloudConstIterator = typename pcl::PointCloud<PointT>::const_iterator;

#endif  // CLOUD_ITERATOR_HPP_
