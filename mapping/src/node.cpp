// Copyright 2022 Takeshi Ishita (2022)
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
//    * Neither the name of the Takeshi Ishita (2022) nor the names of its
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


#include <rclcpp/rclcpp.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <memory>
#include <string>

#include "lidar_feature_library/point_type.hpp"
#include "lidar_feature_library/ros_msg.hpp"
#include "lidar_feature_library/transform.hpp"


using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Exact = message_filters::sync_policies::ExactTime<PointCloud2, PointCloud2, PoseStamped>;
using Synchronizer = message_filters::Synchronizer<Exact>;

const rmw_qos_profile_t qos_profile = rclcpp::SensorDataQoS().keep_last(1).get_rmw_qos_profile();

Eigen::Affine3d GetAffine(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Affine3d transform;
  tf2::fromMsg(pose, transform);
  return transform;
}

template<typename T>
class Map
{
public:
  void TransformAdd(
    const Eigen::Affine3d & transform,
    const typename pcl::PointCloud<T>::Ptr & cloud)
  {
    *map_ptr_ += TransformPointCloud<T>(transform, cloud);
  }

  void Save(const std::string & pcd_filename) const
  {
    pcl::io::savePCDFileASCII(pcd_filename, *map_ptr_);
  }

  typename pcl::PointCloud<T>::Ptr map_ptr_;
};

class MapBuilder
{
public:
  void Callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & edge_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & surface_msg,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr & pose_msg)
  {
    const Eigen::Affine3d transform = GetAffine(pose_msg->pose);
    const pcl::PointCloud<PointXYZIR>::Ptr edge_cloud = getPointCloud<PointXYZIR>(*edge_msg);
    const pcl::PointCloud<PointXYZIR>::Ptr surface_cloud = getPointCloud<PointXYZIR>(*surface_msg);
    edge_map_.TransformAdd(transform, edge_cloud);
    surface_map_.TransformAdd(transform, surface_cloud);
  }

  Map<PointXYZIR> edge_map_;
  Map<PointXYZIR> surface_map_;
};

class FeatureMapping : public rclcpp::Node
{
public:
  explicit FeatureMapping(MapBuilder & builder)
  : rclcpp::Node("lidar_feature_mapping"),
    edge_subscriber_(this, "scan_edge", qos_profile),
    surface_subscriber_(this, "scan_surface", qos_profile),
    pose_subscriber_(this, "pose", qos_profile)
  {
    sync_ = std::make_shared<Synchronizer>(
      Exact(10), edge_subscriber_, surface_subscriber_, pose_subscriber_);

    sync_->registerCallback(
      std::bind(
        &MapBuilder::Callback, builder,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    RCLCPP_INFO(this->get_logger(), "FeatureMapping constructor called");
  }

private:
  message_filters::Subscriber<PointCloud2> edge_subscriber_;
  message_filters::Subscriber<PointCloud2> surface_subscriber_;
  message_filters::Subscriber<PoseStamped> pose_subscriber_;
  std::shared_ptr<Synchronizer> sync_;
};

int main(int argc, char * argv[])
{
  MapBuilder builder;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeatureMapping>(builder));
  rclcpp::shutdown();

  return 0;
}
