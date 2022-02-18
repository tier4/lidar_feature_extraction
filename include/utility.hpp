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
#include "index_range.hpp"
#include "mask.hpp"
#include "math.hpp"
#include "neighbor.hpp"
#include "range.hpp"
#include "ring.hpp"
#include "label.hpp"

struct PointXYZIR
{
  PCL_ADD_POINT4D PCL_ADD_INTENSITY;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIR,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)
)

template<typename T>
pcl::PointXYZ MakePointXYZ(const T & input)
{
  return pcl::PointXYZ(input.x, input.y, input.z);
}

template<typename T>
sensor_msgs::msg::PointCloud2 toRosMsg(const pcl::PointCloud<T> & pointcloud)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pointcloud, msg);
  return msg;
}

template<typename T>
sensor_msgs::msg::PointCloud2 toRosMsg(
  const pcl::PointCloud<T> & pointcloud,
  const rclcpp::Time stamp,
  const std::string frame)
{
  sensor_msgs::msg::PointCloud2 msg = toRosMsg(pointcloud);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  return msg;
}

inline pcl::PointXYZ MakePointXYZ(const Eigen::Vector3d & v)
{
  return pcl::PointXYZ(v(0), v(1), v(2));
}

template<typename T>
typename pcl::PointCloud<T>::Ptr getPointCloud(const sensor_msgs::msg::PointCloud2 & roscloud)
{
  typename pcl::PointCloud<T>::Ptr pclcloud(new pcl::PointCloud<T>());
  pcl::fromROSMsg(roscloud, *pclcloud);
  return pclcloud;
}

template<typename T>
typename pcl::PointCloud<T>::Ptr downsample(
  const typename pcl::PointCloud<T>::Ptr & input_cloud, const float leaf_size)
{
  pcl::VoxelGrid<T> filter;
  typename pcl::PointCloud<T>::Ptr downsampled(new pcl::PointCloud<T>());

  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.setInputCloud(input_cloud);
  filter.filter(*downsampled);

  return downsampled;
}

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

template<typename Element>
std::vector<CurvatureLabel> AssignLabels(
  const ConstReferenceVector<Element> & ref_points,
  const int n_blocks)
{
  const int padding = 5;
  const int max_edges_per_block = 20;
  const double radian_threshold = 2.0;
  const double distance_diff_threshold = 0.3;
  const double range_ratio_threshold = 0.02;
  const double edge_threshold = 0.1;
  const double surface_threshold = 0.1;

  const Neighbor<Element> neighbor(ref_points, radian_threshold);
  const Range<Element> range(ref_points);

  Mask<Element> mask(ref_points, radian_threshold);
  MaskOccludedPoints<Element>(mask, neighbor, range, padding, distance_diff_threshold);
  MaskParallelBeamPoints<Element>(mask, range, range_ratio_threshold);

  return AssignLabel(
    mask, range, n_blocks, padding,
    max_edges_per_block, edge_threshold, surface_threshold);
}

#endif  // UTILITY_HPP_
