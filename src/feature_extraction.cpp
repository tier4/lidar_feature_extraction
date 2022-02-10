// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "utility.hpp"

#include <range/v3/all.hpp>

#include <algorithm>
#include <deque>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

//  VLS-128 Lidar Sensor Configuration
const int N_SCAN = 128;
const int HORIZONTAL_SIZE = 1800;
const float range_min = 1.0;
const float range_max = 1000.0;

//  LOAM
const float edgeThreshold = 0.1;
const float surfThreshold = 0.1;

//  voxel filter paprams
const float surface_leaf_size = 0.2;
const float map_edge_leaf_size = 0.2;
const float map_surface_leaf_size = 0.2;

const int N_BLOCKS = 6;

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
    const pcl::PointCloud<PointXYZIR> input_points = *getPointCloud<PointXYZIR>(*cloud_msg);
    RCLCPP_INFO(
      this->get_logger(),
      "x = %f,  y = %f,  z = %f,  intensity = %f,  ring = %u",
      input_points.at(0).x,
      input_points.at(0).y,
      input_points.at(0).z,
      input_points.at(0).intensity,
      static_cast<unsigned int>(input_points.at(0).ring));

    if (!input_points.is_dense) {
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

    auto point_to_index = [&](const PointXYZIR & p) {
        const int column_index = ColumnIndex(HORIZONTAL_SIZE, p.x, p.y);
        return CalcIndex(HORIZONTAL_SIZE, p.ring, column_index);
      };

    const pcl::PointCloud<PointXYZIR> filtered = FilterByRange(input_points, range_min, range_max);
    const auto output_points = ExtractElements<PointXYZIR>(point_to_index, filtered);

    std::vector<int> column_indices(N_SCAN * HORIZONTAL_SIZE, 0);
    pcl::PointCloud<pcl::PointXYZ> cloud(output_points.size(), 1);
    std::vector<IndexRange> index_ranges(N_SCAN);

    int count = 0;
    for (int row_index = 0; row_index < N_SCAN; ++row_index) {
      const int start_index = count + 5;

      for (int column_index = 0; column_index < HORIZONTAL_SIZE; ++column_index) {
        const int index = column_index + row_index * HORIZONTAL_SIZE;
        if (output_points.find(index) == output_points.end()) {
          continue;
        }

        column_indices.at(count) = column_index;
        cloud.at(count) = MakePointXYZ(output_points.at(index));
        count += 1;
      }

      const int end_index = count - 5;

      const IndexRange index_range(start_index, end_index, N_BLOCKS);
      index_ranges.push_back(index_range);
    }

    auto calc_norm = [](const pcl::PointXYZ & p) {
        return Eigen::Vector3d(p.x, p.y, p.z).norm();
      };

    const std::vector<double> range = cloud |
      ranges::views::transform(calc_norm) |
      ranges::to_vector;

    // used to prevent from labeling a neighbor as surface or edge
    std::vector<bool> mask(N_SCAN * HORIZONTAL_SIZE);

    for (unsigned int i = 5; i < cloud.size() - 5; i++) {
      mask.at(i) = false;
    }

    MaskOccludedPoints(column_indices, range, mask);
    MaskParallelBeamPoints(range, mask);

    auto [curvature, inds] = CalcCurvature(range, N_SCAN, HORIZONTAL_SIZE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr edge(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface(new pcl::PointCloud<pcl::PointXYZ>());

    std::vector<CurvatureLabel> label(N_SCAN * HORIZONTAL_SIZE, CurvatureLabel::Default);

    for (const IndexRange & index_range : index_ranges) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr surface_scan(new pcl::PointCloud<pcl::PointXYZ>());

      for (int j = 0; j < N_BLOCKS; j++) {
        const int sp = index_range.Begin(j);
        const int ep = index_range.End(j);
        std::sort(inds.begin() + sp, inds.begin() + ep, by_value(curvature));

        int n_picked = 0;
        for (int k = ep; k >= sp; k--) {
          const int index = inds.at(k);
          if (mask.at(index) || curvature.at(index) <= edgeThreshold) {
            continue;
          }

          if (n_picked >= 20) {
            break;
          }

          n_picked++;

          edge->push_back(cloud.at(index));
          label.at(index) = CurvatureLabel::Edge;

          NeighborPicked(column_indices, index, mask);
        }

        for (int k = sp; k <= ep; k++) {
          const int index = inds.at(k);
          if (mask.at(index) || curvature.at(index) >= surfThreshold) {
            continue;
          }

          label.at(index) = CurvatureLabel::Surface;

          NeighborPicked(column_indices, index, mask);
        }

        for (int k = sp; k <= ep; k++) {
          if (label.at(k) == CurvatureLabel::Default || label.at(k) == CurvatureLabel::Edge) {
            surface_scan->push_back(cloud.at(k));
          }
        }
      }

      *surface += *downsample<pcl::PointXYZ>(surface_scan, surface_leaf_size);
    }

    const auto edge_downsampled = downsample<pcl::PointXYZ>(edge, map_edge_leaf_size);
    const auto surface_downsampled = downsample<pcl::PointXYZ>(surface, map_surface_leaf_size);

    const std::string lidar_frame = "base_link";
    const auto cloud_edge = toRosMsg(*edge_downsampled, cloud_msg->header.stamp, lidar_frame);
    const auto cloud_surface = toRosMsg(*surface_downsampled, cloud_msg->header.stamp, lidar_frame);
    edge_publisher_->publish(cloud_edge);
    surface_publisher_->publish(cloud_surface);
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
