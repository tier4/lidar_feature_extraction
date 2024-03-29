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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/auto_io.h>

#include <string>
#include <memory>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "lidar_feature_library/qos.hpp"
#include "lidar_feature_library/ros_msg.hpp"


sensor_msgs::msg::PointCloud2 LoadMap(const std::string & filepath)
{
  sensor_msgs::msg::PointCloud2 map;
  pcl::io::loadPCDFile(filepath, map);
  map.header.frame_id = "map";
  return map;
}

class MapLoaderNode : public rclcpp::Node
{
public:
  explicit MapLoaderNode(const rclcpp::NodeOptions & options)
  : Node("map_loader", options)
  {
    const std::string pcd_filename = this->declare_parameter("pcd_filename", "");

    if (pcd_filename.size() == 0) {
      throw std::invalid_argument("pcd_filename is not set");
    }

    const sensor_msgs::msg::PointCloud2 map = LoadMap(pcd_filename);

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded point cloud map from %s of size (width = %u, height = %u)",
      pcd_filename.c_str(), map.width, map.height);

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "map_topic", QOS_RELIABLE_TRANSIENT_LOCAL);
    publisher_->publish(map);

    RCLCPP_INFO(this->get_logger(), "The map point cloud has been published");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(MapLoaderNode)
