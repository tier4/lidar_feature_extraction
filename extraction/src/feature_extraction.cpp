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
#include "lidar_feature_extraction/curvature.hpp"
#include "lidar_feature_extraction/index_range.hpp"
#include "lidar_feature_extraction/label.hpp"
#include "lidar_feature_extraction/math.hpp"
#include "lidar_feature_extraction/neighbor.hpp"
#include "lidar_feature_extraction/point_label.hpp"
#include "lidar_feature_extraction/range.hpp"
#include "lidar_feature_extraction/ring.hpp"

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
    padding_(this->declare_parameter("convolution_padding", 5)),
    max_edges_per_block_(this->declare_parameter("max_edges_per_block", 20)),
    radian_threshold_(this->declare_parameter("radian_threshold", 2.0)),
    distance_diff_threshold_(this->declare_parameter("distance_diff_threshold", 0.3)),
    range_ratio_threshold_(this->declare_parameter("range_ratio_threshold", 0.02)),
    edge_threshold_(this->declare_parameter("edge_threshold", 0.05)),
    surface_threshold_(this->declare_parameter("surface_threshold", 0.05)),
    min_range_(this->declare_parameter("min_range_", 0.1)),
    max_range_(this->declare_parameter("max_range_", 100.0)),
    edge_label_(padding_, edge_threshold_, max_edges_per_block_),
    surface_label_(padding_, surface_threshold_),
    cloud_subscriber_(this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_raw", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&FeatureExtraction::Callback, this, std::placeholders::_1),
        this->MakeSubscriptionOption())),
    edge_publisher_(this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_edge", 1)),
    surface_publisher_(this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_surface", 1))
  {
    RCLCPP_INFO(this->get_logger(), "edge_threshold_ : %lf", edge_threshold_);
    RCLCPP_INFO(this->get_logger(), "surface_threshold_ : %lf", surface_threshold_);
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
        const MappedPoints<PointXYZIR> ref_points(input_points, indices);

        const Neighbor<PointXYZIR> neighbor(ref_points, radian_threshold_);
        const Range<PointXYZIR> range(ref_points);

        Label<PointXYZIR> label(ref_points, radian_threshold_);
        LabelOutOfRange(label, range, min_range_, max_range_);
        LabelOccludedPoints(label, neighbor, range, padding_, distance_diff_threshold_);
        LabelParallelBeamPoints(label, range, range_ratio_threshold_);

        const std::vector<PointLabel> labels = AssignLabel(
          label, range, edge_label_, surface_label_, n_blocks, padding_);

        ExtractByLabel<PointXYZIR>(edge, ref_points, labels, PointLabel::Edge);
        ExtractByLabel<PointXYZIR>(surface, ref_points, labels, PointLabel::Surface);
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

  const int padding_;
  const int max_edges_per_block_;
  const double radian_threshold_;
  const double distance_diff_threshold_;
  const double range_ratio_threshold_;
  const double edge_threshold_;
  const double surface_threshold_;
  const double min_range_;
  const double max_range_;
  const EdgeLabel<PointXYZIR> edge_label_;
  const SurfaceLabel<PointXYZIR> surface_label_;
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
