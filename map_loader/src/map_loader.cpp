// Copyright 2022 Takeshi Ishita
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
//    * Neither the name of the Takeshi Ishita nor the names of its
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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/auto_io.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <string>
#include <memory>

#include "lidar_feature_library/point_type.hpp"
#include "lidar_feature_library/ros_msg.hpp"


const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

sensor_msgs::msg::PointCloud2 LoadMap(const std::string & filepath)
{
  sensor_msgs::msg::PointCloud2 map;
  pcl::io::loadPCDFile(filepath, map);
  map.header.frame_id = "map";
  return map;
}

class MapPublisherNode : public rclcpp::Node
{
public:
  MapPublisherNode(
    const std::string & node_name,
    const std::string & topic_name,
    const std::string & pcd_filename)
  : Node(node_name),
    publisher_(this->MakePublisher<sensor_msgs::msg::PointCloud2>(topic_name))
  {
    const auto map = LoadMap(pcd_filename);
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded point cloud map from %s of size (width = %u, height = %u)",
      pcd_filename.c_str(), map.width, map.height);
    publisher_->publish(map);
  }

private:
  template<typename T>
  typename rclcpp::Publisher<T>::SharedPtr MakePublisher(const std::string & topic_name)
  {
    return this->create_publisher<T>(topic_name, qos);
  }

  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  const auto edge_node = std::make_shared<MapPublisherNode>(
    "edge_map_publisher", "/edge_map", "maps/edge.pcd");
  const auto surface_node = std::make_shared<MapPublisherNode>(
    "surface_map_publisher", "/surface_map", "maps/surface.pcd");
  exec.add_node(edge_node);
  exec.add_node(surface_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
