// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _CLOUD_ITERATOR_LIDAR_ODOMETRY_H_
#define _CLOUD_ITERATOR_LIDAR_ODOMETRY_H_

template<typename PointT>
using CloudIterator = typename pcl::PointCloud<PointT>::iterator;

template<typename PointT>
using CloudConstIterator = typename pcl::PointCloud<PointT>::const_iterator;

#endif  /* _CLOUD_ITERATOR_LIDAR_ODOMETRY_H_ */
