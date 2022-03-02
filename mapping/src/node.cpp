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

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <memory>
#include <string>

#include "lidar_feature_library/point_type.hpp"
#include "lidar_feature_library/ros_msg.hpp"

#include "lidar_feature_mapping/map.hpp"


using Exact = message_filters::sync_policies::ExactTime<
  sensor_msgs::msg::PointCloud2,
  geometry_msgs::msg::PoseStamped>;
using Synchronizer = message_filters::Synchronizer<Exact>;

const rmw_qos_profile_t qos_profile = rclcpp::SensorDataQoS().keep_last(1).get_rmw_qos_profile();

class MapSubscriber : public rclcpp::Node
{
public:
  MapSubscriber(
    std::shared_ptr<MapBuilder> & builder,
    const std::string & cloud_topic_name,
    const std::string & pose_topic_name)
  : rclcpp::Node("lidar_feature_mapping"),
    cloud_subscriber_(this, cloud_topic_name, qos_profile),
    pose_subscriber_(this, pose_topic_name, qos_profile),
    sync_(std::make_shared<Synchronizer>(Exact(10), cloud_subscriber_, pose_subscriber_))
  {
    sync_->registerCallback(
      std::bind(&MapBuilder::Callback, builder, std::placeholders::_1, std::placeholders::_2));
  }

private:
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_subscriber_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_subscriber_;
  std::shared_ptr<Synchronizer> sync_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  RCLCPP_DEBUG(rclcpp::get_logger("lidar_feature_mapping"), "Start nodes");

  auto edge_map_builder = std::make_shared<MapBuilder>();
  auto surface_map_builder = std::make_shared<MapBuilder>();

  auto edge_sub = std::make_shared<MapSubscriber>(
    edge_map_builder, "scan_edge", "pose");
  auto surface_sub = std::make_shared<MapSubscriber>(
    surface_map_builder, "scan_surface", "pose");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(edge_sub);
  exec.add_node(surface_sub);
  exec.spin();

  edge_map_builder->SaveMap("edge.pcd");
  surface_map_builder->SaveMap("surface.pcd");

  rclcpp::shutdown();

  return 0;
}
