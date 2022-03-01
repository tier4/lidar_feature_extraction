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

#include "lidar_feature_extraction/cloud_iterator.hpp"
#include "lidar_feature_extraction/curvature_label.hpp"
#include "lidar_feature_extraction/curvature.hpp"
#include "lidar_feature_extraction/index_range.hpp"
#include "lidar_feature_extraction/mask.hpp"
#include "lidar_feature_extraction/math.hpp"
#include "lidar_feature_extraction/neighbor.hpp"
#include "lidar_feature_extraction/range.hpp"
#include "lidar_feature_extraction/ring.hpp"
#include "lidar_feature_extraction/label.hpp"

#include "lidar_feature_library/ros_msg.hpp"

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
  : Node("lidar_feature_extraction"),
    cloud_subscriber_(this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_raw", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&FeatureExtraction::Callback, this, std::placeholders::_1),
        this->MakeSubscriptionOption())),
    edge_publisher_(this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_edge", 1)),
    surface_publisher_(this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_surface", 1))
  {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  }

  ~FeatureExtraction() {}

private:
  rclcpp::SubscriptionOptions MakeSubscriptionOption()
  {
    const rclcpp::CallbackGroup::SharedPtr callback_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto main_sub_opt = rclcpp::SubscriptionOptions();
    main_sub_opt.callback_group = callback_group;
    return main_sub_opt;
  }

  void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) const
  {
    const pcl::PointCloud<PointXYZIR>::Ptr input_points = getPointCloud<PointXYZIR>(*cloud_msg);

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

    const auto rings = ExtractAngleSortedRings(*input_points);

    pcl::PointCloud<PointXYZIR>::Ptr edge(new pcl::PointCloud<PointXYZIR>());
    pcl::PointCloud<PointXYZIR>::Ptr surface(new pcl::PointCloud<PointXYZIR>());

    for (const auto & [ring, indices] : rings) {
      try {
        const MappedPoints<PointXYZIR> wrapper(input_points, indices);
        const std::vector<CurvatureLabel> labels = AssignLabels<PointXYZIR>(wrapper, n_blocks);
        ExtractByLabel<PointXYZIR>(edge, wrapper, labels, CurvatureLabel::Edge);
        ExtractByLabel<PointXYZIR>(surface, wrapper, labels, CurvatureLabel::Surface);
      } catch (const std::invalid_argument & e) {
        RCLCPP_WARN(this->get_logger(), e.what());
      }
    }

    const std::string lidar_frame = "base_link";
    const auto cloud_edge = toRosMsg<PointXYZIR>(edge, cloud_msg->header.stamp, lidar_frame);
    const auto cloud_surface = toRosMsg<PointXYZIR>(surface, cloud_msg->header.stamp, lidar_frame);
    edge_publisher_->publish(cloud_edge);
    surface_publisher_->publish(cloud_surface);
  }

  const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge_publisher_;
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeatureExtraction>());
  rclcpp::shutdown();
  return 0;
}
