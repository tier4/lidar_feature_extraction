// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef ROS_MSG_HPP_
#define ROS_MSG_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <string>

template<typename T>
sensor_msgs::msg::PointCloud2 toRosMsg(const typename pcl::PointCloud<T>::Ptr & cloud_ptr)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud_ptr, msg);
  return msg;
}

template<typename T>
sensor_msgs::msg::PointCloud2 toRosMsg(
  const typename pcl::PointCloud<T>::Ptr & cloud_ptr,
  const rclcpp::Time stamp,
  const std::string frame)
{
  sensor_msgs::msg::PointCloud2 msg = toRosMsg<T>(cloud_ptr);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  return msg;
}

template<typename T>
typename pcl::PointCloud<T>::Ptr getPointCloud(const sensor_msgs::msg::PointCloud2 & roscloud)
{
  typename pcl::PointCloud<T>::Ptr pclcloud(new pcl::PointCloud<T>());
  pcl::fromROSMsg(roscloud, *pclcloud);
  return pclcloud;
}

#endif   // ROS_MSG_HPP_
