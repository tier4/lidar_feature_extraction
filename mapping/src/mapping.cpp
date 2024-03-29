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


#include <Eigen/Geometry>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "lidar_feature_library/qos.hpp"
#include "lidar_feature_mapping/map.hpp"


using Exact = message_filters::sync_policies::ExactTime<
  sensor_msgs::msg::PointCloud2,
  geometry_msgs::msg::PoseStamped>;
using Synchronizer = message_filters::Synchronizer<Exact>;
using PointType = PointXYZCR;


class MapSubscriber : public rclcpp::Node
{
public:
  MapSubscriber(
    std::shared_ptr<MapBuilder<PointType>> & builder,
    const std::string & node_name,
    const std::string & cloud_topic_name,
    const std::string & pose_topic_name)
  : rclcpp::Node(node_name),
    cloud_subscriber_(this, cloud_topic_name, QOS_RELIABLE_VOLATILE.get_rmw_qos_profile()),
    pose_subscriber_(this, pose_topic_name, QOS_RELIABLE_VOLATILE.get_rmw_qos_profile()),
    sync_(std::make_shared<Synchronizer>(Exact(1000), cloud_subscriber_, pose_subscriber_))
  {
    sync_->registerCallback(
      std::bind(
        &MapBuilder<PointType>::Callback, builder,
        std::placeholders::_1, std::placeholders::_2));
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

  auto edge_map_builder = std::make_shared<MapBuilder<PointType>>();

  auto edge_sub = std::make_shared<MapSubscriber>(
    edge_map_builder, "edge_map_builder", "scan_edge", "pose");

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(edge_sub);
  exec.spin();

  edge_map_builder->SaveMap("maps/edge.pcd");

  rclcpp::shutdown();

  return 0;
}
