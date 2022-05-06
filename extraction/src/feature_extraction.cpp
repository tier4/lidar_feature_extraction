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
#include "lidar_feature_extraction/color_points.hpp"
#include "lidar_feature_extraction/curvature.hpp"
#include "lidar_feature_extraction/index_range.hpp"
#include "lidar_feature_extraction/label.hpp"
#include "lidar_feature_extraction/math.hpp"
#include "lidar_feature_extraction/neighbor.hpp"
#include "lidar_feature_extraction/occlusion.hpp"
#include "lidar_feature_extraction/out_of_range.hpp"
#include "lidar_feature_extraction/parallel_beam.hpp"
#include "lidar_feature_extraction/point_label.hpp"
#include "lidar_feature_extraction/range.hpp"
#include "lidar_feature_extraction/ring.hpp"

#include "lidar_feature_library/degree_to_radian.hpp"
#include "lidar_feature_library/ros_msg.hpp"


rclcpp::SubscriptionOptions MutuallyExclusiveOption(rclcpp::Node & node)
{
  const rclcpp::CallbackGroup::SharedPtr callback_group =
    node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto main_sub_opt = rclcpp::SubscriptionOptions();
  main_sub_opt.callback_group = callback_group;
  return main_sub_opt;
}

struct HyperParameters
{
  explicit HyperParameters(rclcpp::Node & node)
  : padding(node.declare_parameter("convolution_padding", 5)),
    neighbor_degree_threshold(node.declare_parameter("neighbor_degree_threshold", 2.0)),
    distance_diff_threshold(node.declare_parameter("distance_diff_threshold", 0.3)),
    range_ratio_threshold(node.declare_parameter("range_ratio_threshold", 0.02)),
    edge_threshold(node.declare_parameter("edge_threshold", 0.05)),
    surface_threshold(node.declare_parameter("surface_threshold", 0.05)),
    min_range(node.declare_parameter("min_range", 0.1)),
    max_range(node.declare_parameter("max_range", 100.0)),
    n_blocks(node.declare_parameter("n_blocks", 6))
  {
    assert(padding > 0);
    assert(neighbor_degree_threshold > 0);
    assert(distance_diff_threshold > 0);
    assert(range_ratio_threshold > 0);
    assert(edge_threshold > 0);
    assert(surface_threshold > 0);
    assert(min_range > 0);
    assert(max_range > 0);
    assert(n_blocks > 0);
  }

  const int padding;
  const double neighbor_degree_threshold;
  const double distance_diff_threshold;
  const double range_ratio_threshold;
  const double edge_threshold;
  const double surface_threshold;
  const double min_range;
  const double max_range;
  const int n_blocks;
};

class FeatureExtraction : public rclcpp::Node
{
public:
  FeatureExtraction()
  : Node("lidar_feature_extraction"),
    params_(HyperParameters(*this)),
    edge_label_(params_.padding, params_.edge_threshold),
    surface_label_(params_.padding, params_.surface_threshold),
    cloud_subscriber_(
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_raw", rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&FeatureExtraction::Callback, this, std::placeholders::_1),
        MutuallyExclusiveOption(*this))),
    colored_scan_publisher_(
      this->create_publisher<sensor_msgs::msg::PointCloud2>("colored_scan", 1)),
    curvature_cloud_publisher_(
      this->create_publisher<sensor_msgs::msg::PointCloud2>("curvature_scan", 1)),
    edge_publisher_(this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_edge", 1)),
    surface_publisher_(this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_surface", 1))
  {
    RCLCPP_INFO(this->get_logger(), "edge_threshold_ : %lf", params_.edge_threshold);
    RCLCPP_INFO(this->get_logger(), "surface_threshold_ : %lf", params_.surface_threshold);
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  }

  ~FeatureExtraction() {}

private:
  void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) const
  {
    const pcl::PointCloud<PointXYZIR>::Ptr input_cloud = GetPointCloud<PointXYZIR>(*cloud_msg);

    if (!input_cloud->is_dense) {
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

    const double debug_max_curvature = 10.;

    pcl::PointCloud<PointXYZIR>::Ptr edge(new pcl::PointCloud<PointXYZIR>());
    pcl::PointCloud<PointXYZIR>::Ptr surface(new pcl::PointCloud<PointXYZIR>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr curvature_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    const auto rings = [&] {
        auto rings = ExtractAngleSortedRings(*input_cloud);
        RemoveInsufficientNumRing(rings, params_.padding + 1);
        return rings;
      } ();

    for (const auto & [ring, indices] : rings) {
      const MappedPoints<PointXYZIR> ref_points(input_cloud, indices);
      const double radian_threshold = DegreeToRadian(params_.neighbor_degree_threshold);
      const NeighborCheckXY<PointXYZIR> is_neighbor(ref_points, radian_threshold);
      const Range<PointXYZIR> range(ref_points);

      try {
        std::vector<PointLabel> labels = InitLabels(ref_points.Size());

        const std::vector<double> ranges = range(0, range.Size());
        const std::vector<double> curvature = CalcCurvature(ranges, params_.padding);
        const PaddedIndexRange index_range(range.Size(), params_.n_blocks, params_.padding);

        AssignLabel(labels, curvature, is_neighbor, index_range, edge_label_, surface_label_);

        LabelOccludedPoints(
          labels, is_neighbor, range,
          params_.padding, params_.distance_diff_threshold);
        LabelOutOfRange(labels, range, params_.min_range, params_.max_range);
        LabelParallelBeamPoints(labels, range, params_.range_ratio_threshold);

        ExtractByLabel<PointXYZIR>(edge, ref_points, labels, PointLabel::Edge);
        ExtractByLabel<PointXYZIR>(surface, ref_points, labels, PointLabel::Surface);

        *colored_cloud += *ColorPointsByLabel<PointXYZIR>(ref_points, labels);
        *curvature_cloud += *ColorPointsByValue(ref_points, curvature, 0., debug_max_curvature);
      } catch (const std::invalid_argument & e) {
        RCLCPP_WARN(this->get_logger(), e.what());
      }
    }

    const std::string lidar_frame = "base_link";
    const auto stamp = cloud_msg->header.stamp;
    const auto colored_msg = ToRosMsg<pcl::PointXYZRGB>(colored_cloud, stamp, lidar_frame);
    const auto curvature_msg = ToRosMsg<pcl::PointXYZRGB>(curvature_cloud, stamp, lidar_frame);
    const auto cloud_edge = ToRosMsg<PointXYZIR>(edge, stamp, lidar_frame);
    const auto cloud_surface = ToRosMsg<PointXYZIR>(surface, stamp, lidar_frame);
    colored_scan_publisher_->publish(colored_msg);
    curvature_cloud_publisher_->publish(curvature_msg);
    edge_publisher_->publish(cloud_edge);
    surface_publisher_->publish(cloud_surface);
  }

  const HyperParameters params_;
  const EdgeLabel edge_label_;
  const SurfaceLabel surface_label_;
  const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_scan_publisher_;
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr curvature_cloud_publisher_;
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
