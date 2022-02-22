// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <range/v3/all.hpp>

#include <algorithm>
#include <deque>
#include <functional>
#include <iterator>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "cloud_iterator.hpp"
#include "curvature_label.hpp"
#include "curvature.hpp"
#include "downsample.hpp"
#include "extraction.hpp"
#include "index_range.hpp"
#include "mask.hpp"
#include "math.hpp"
#include "neighbor.hpp"
#include "range.hpp"
#include "ros_msg.hpp"
#include "ring.hpp"
#include "label.hpp"

//  VLS-128 Lidar Sensor Configuration
const int N_SCAN = 128;
const int HORIZONTAL_SIZE = 1800;
const float range_min = 1.0;
const float range_max = 1000.0;

//  voxel filter paprams
const float surface_leaf_size = 0.2;
const float map_edge_leaf_size = 0.2;
const float map_surface_leaf_size = 0.2;

const int n_blocks = 6;

//  CPU Params
const int n_cores = 2;


class FeatureExtraction : public rclcpp::Node
{
public:
  FeatureExtraction()
  : Node("lidar_feature_extraction")
  {
    rclcpp::CallbackGroup::SharedPtr main_callback_group;
    main_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto main_sub_opt = rclcpp::SubscriptionOptions();
    main_sub_opt.callback_group = main_callback_group;

    cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_raw", rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&FeatureExtraction::Callback, this, std::placeholders::_1), main_sub_opt);
    edge_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_edge", 1);
    surface_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_surface", 1);
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  }

  ~FeatureExtraction() {}

private:
  void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {
    const pcl::PointCloud<PointXYZIR>::Ptr input_points = getPointCloud<PointXYZIR>(*cloud_msg);

    RCLCPP_INFO(
      this->get_logger(), "width = %d, height = %d",
      input_points->width, input_points->height);

    if (!input_points->is_dense) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Point cloud is not in dense format, please remove NaN points first!");
      rclcpp::shutdown();
    }

    if (!RingIsAvailable(cloud_msg->fields)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Point cloud ring channel could not be found");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Ring extraction start");

    const auto rings = ExtractAngleSortedRings(*input_points);

    RCLCPP_INFO(this->get_logger(), "Ring extraction finished");

    RCLCPP_INFO(this->get_logger(), "Point labeling start");

    pcl::PointCloud<PointXYZIR>::Ptr edge(new pcl::PointCloud<PointXYZIR>());
    pcl::PointCloud<PointXYZIR>::Ptr surface(new pcl::PointCloud<PointXYZIR>());

    for (const auto & [ring, indices] : rings) {
      RCLCPP_INFO(
        this->get_logger(),
        "ring = %d, indices.size() = %ld", ring, indices.size());
      RCLCPP_INFO(this->get_logger(), "Assign labels");
      const MappedPoints<PointXYZIR> wrapper(*input_points, indices);
      const std::vector<CurvatureLabel> labels = AssignLabels<PointXYZIR>(wrapper, n_blocks);

      // RCLCPP_INFO(this->get_logger(), "Extract by label");
      // ExtractByLabel<PointXYZIR>(edge, wrapper, labels, CurvatureLabel::Edge);
      // ExtractByLabel<PointXYZIR>(surface, wrapper, labels, CurvatureLabel::Surface);
    }

    RCLCPP_INFO(this->get_logger(), "Point labeling finished");

    /*
    // const auto edge_downsampled = downsample<PointXYZIR>(edge, map_edge_leaf_size);
    // const auto surface_downsampled = downsample<PointXYZIR>(surface, map_surface_leaf_size);

    const std::string lidar_frame = "base_link";
    const auto cloud_edge = toRosMsg(edge, cloud_msg->header.stamp, lidar_frame);
    const auto cloud_surface = toRosMsg(surface, cloud_msg->header.stamp, lidar_frame);
    edge_publisher_->publish(cloud_edge);
    surface_publisher_->publish(cloud_surface);
    */
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeatureExtraction>());
  rclcpp::shutdown();
  return 0;
}
