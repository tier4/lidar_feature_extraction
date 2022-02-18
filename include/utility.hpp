// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef UTILITY_HPP_
#define UTILITY_HPP_

#include <boost/range/adaptor/reversed.hpp>

#include <fmt/core.h>

#include <std_msgs/msg/header.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>

#include <range/v3/all.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "cloud_iterator.hpp"
#include "curvature_label.hpp"
#include "curvature.hpp"
#include "downsample.hpp"
#include "index_range.hpp"
#include "mask.hpp"
#include "math.hpp"
#include "neighbor.hpp"
#include "range.hpp"
#include "ring.hpp"
#include "label.hpp"
#include "point_type.hpp"

template<typename PointT>
pcl::PointCloud<PointT> ExtractEdge(
  const CloudConstIterator<PointT> cloud_begin,
  const std::vector<CurvatureLabel> & labels)
{
  typename pcl::PointCloud<PointT>::Ptr edge(new pcl::PointCloud<pcl::PointXYZ>());
  for (unsigned int i = 0; i < labels.size(); i++) {
    if (labels[i] == CurvatureLabel::Edge) {
      const PointT point = *(cloud_begin + i);
      edge->push_back(point);
    }
  }
  return edge;
}

#endif  // UTILITY_HPP_
