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
#include "lidar_feature_extraction/hyper_parameter.hpp"
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
#include "lidar_feature_extraction/subscription.hpp"

#include "lidar_feature_library/convert_point_cloud_type.hpp"
#include "lidar_feature_library/degree_to_radian.hpp"
#include "lidar_feature_library/ros_msg.hpp"

const rclcpp::QoS qos_keep_all = rclcpp::SensorDataQoS().keep_all().reliable();

template<typename PointType>
typename pcl::PointCloud<PointType>::Ptr FilterByCoordinate(
  const typename pcl::PointCloud<PointType>::Ptr & edge)
{
  typename pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
  for (const PointType & p : *edge) {
    if (-5. < p.y && p.y < 1.0) {
      continue;
    }
    filtered->push_back(p);
  }
  return filtered;
}

class FeatureExtraction : public rclcpp::Node
{
public:
  FeatureExtraction()
  : Node("lidar_feature_extraction"),
    params_(HyperParameters(*this)),
    edge_label_(params_.padding, params_.edge_threshold),
    cloud_subscriber_(
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_raw", qos_keep_all,
        std::bind(&FeatureExtraction::Callback, this, std::placeholders::_1))),
    colored_scan_publisher_(
      this->create_publisher<sensor_msgs::msg::PointCloud2>("colored_scan", 1)),
    curvature_cloud_publisher_(
      this->create_publisher<sensor_msgs::msg::PointCloud2>("curvature_scan", 1)),
    edge_publisher_(
      this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_edge", qos_keep_all))
  {
    RCLCPP_INFO(this->get_logger(), "edge_threshold_ : %lf", params_.edge_threshold);
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  }

  ~FeatureExtraction() {}

private:
  void Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) const
  {
    const auto input_cloud = GetPointCloud<PointXYZIR>(*cloud_msg);

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

    pcl::PointCloud<PointXYZCR>::Ptr edge(new pcl::PointCloud<PointXYZCR>());
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
        std::vector<PointLabel> labels = InitLabels(ref_points.size());

        const std::vector<double> ranges = range(0, range.size());
        const std::vector<double> curvature = CalcCurvature(ranges, params_.padding);
        const PaddedIndexRange index_range(range.size(), params_.n_blocks, params_.padding);

        AssignLabel(labels, curvature, is_neighbor, index_range, edge_label_);

        LabelOccludedPoints(
          labels, is_neighbor, range, params_.padding, params_.distance_diff_threshold);
        LabelOutOfRange(labels, range, params_.min_range, params_.max_range);
        LabelParallelBeamPoints(labels, range, params_.parallel_beam_min_range_ratio);

        assert(curvature.size() == static_cast<size_t>(ref_points.size()));

        const std::vector<size_t> indices = GetIndicesByLabel(labels, PointLabel::Edge);
        const std::vector<PointXYZIR> edge_points =
          indices |
          ranges::views::transform([&](size_t i) {return ref_points.at(i);}) |
          ranges::to_vector;
        const std::vector<double> edge_curvature =
          indices |
          ranges::views::transform([&](size_t i) {return curvature[i];}) |
          ranges::to_vector;

        AppendXYZCR<PointXYZIR>(edge, edge_points, edge_curvature);

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

    const auto filtered = FilterByCoordinate<PointXYZCR>(edge);
    const auto edge_msg = ToRosMsg<PointXYZCR>(filtered, stamp, lidar_frame);

    colored_scan_publisher_->publish(colored_msg);
    curvature_cloud_publisher_->publish(curvature_msg);
    edge_publisher_->publish(edge_msg);
  }

  const HyperParameters params_;
  const EdgeLabel edge_label_;
  const rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_scan_publisher_;
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr curvature_cloud_publisher_;
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeatureExtraction>());
  rclcpp::shutdown();
  return 0;
}
