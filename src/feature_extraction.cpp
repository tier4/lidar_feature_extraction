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

struct VelodynePointXYZIRT
{
  PCL_ADD_POINT4D PCL_ADD_INTENSITY;
  std::uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
  VelodynePointXYZIRT,
  (float, x, x)(float, y, y) (float, z, z) (float, intensity, intensity)(
    std::uint16_t, ring,
    ring) (float, time, time)
)

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

bool ringIsAvailable(const sensor_msgs::msg::PointCloud2 & pointcloud)
{
  for (const auto & field : pointcloud.fields) {
    if (field.name == "ring") {
      return true;
    }
  }
  return false;
}

bool timeStampIsAvailable(const sensor_msgs::msg::PointCloud2 & pointcloud)
{
  for (auto & field : pointcloud.fields) {
    if (field.name == "time" || field.name == "t" || field.name == "time_stamp") {
      return true;
    }
  }
  return false;
}

float rad2deg(const float rad)
{
  return rad * 180 / M_PI;
}

int calcColumnIndex(const int horizontal_size, const double x, const double y)
{
  const double angle = rad2deg(atan2(y, x));
  const double k = horizontal_size * angle / (180.0 * 2.0);
  const double u = k + horizontal_size / 2.0;
  return static_cast<int>(u);
}

std::tuple<std::vector<int>, std::vector<double>, std::vector<Eigen::Vector3d>>
extractElements(
  const pcl::PointCloud<PointXYZIRT> & input_points,
  const float range_min, const float range_max,
  const int horizontal_size)
{
  const auto f = [&](const PointXYZIRT & p) {
      const int row_index = p.ring;
      const int column_index = calcColumnIndex(horizontal_size, p.x, p.y);
      const int index = column_index + row_index * horizontal_size;
      const Eigen::Vector3d q(p.x, p.y, p.z);
      return std::make_tuple(index, p.time, q);
    };

  std::set<int> unique_indices;
  std::vector<int> indices;
  std::vector<double> times;
  std::vector<Eigen::Vector3d> points;

  const auto iterator = input_points | ranges::views::transform(f);
  for (const auto & [index, time, point] : iterator) {
    const double range = point.norm();
    if (range < range_min || range_max < range) {
      continue;
    }

    if (unique_indices.find(index) != unique_indices.end()) {
      continue;
    }

    unique_indices.insert(index);
    indices.push_back(index);
    times.push_back(time);
    points.push_back(point);
  }

  return {indices, times, points};
}

std::unordered_map<int, double> makeRangeMatrix(
  const std::vector<int> & indices,
  const std::vector<Eigen::Vector3d> & points)
{
  std::unordered_map<int, double> range_map;
  for (const auto & [index, point] : ranges::views::zip(indices, points)) {
    range_map[index] = point.norm();
  }
  return range_map;
}

std::unordered_map<int, Eigen::Vector3d> projectWithoutImu(
  const std::vector<int> & indices,
  const std::vector<Eigen::Vector3d> points)
{
  std::unordered_map<int, Eigen::Vector3d> output_points;
  for (const auto & [index, q] : ranges::views::zip(indices, points)) {
    output_points[index] = q;
  }
  return output_points;
}

class by_value
{
public:
  explicit by_value(const std::vector<float> & values)
  : values_(values) {}
  bool operator()(const int & left, const int & right)
  {
    return values_[left] < values_[right];
  }

private:
  std::vector<float> values_;
};

enum class CurvatureLabel
{
  Default = 0,
  Edge = 1,
  Surface = -1
};

bool isNeighbor(const std::vector<int> & column_indices, const int index1, const int index2)
{
  return std::abs(column_indices[index1] - column_indices[index2]) <= 10;
}

void neighborPicked(
  const std::vector<int> & column_indices,
  const int index,
  std::vector<bool> & mask)
{
  mask[index] = true;
  for (int l = 1; l <= 5; l++) {
    if (!isNeighbor(column_indices, index + l, index + l - 1)) {
      break;
    }
    mask[index + l] = true;
  }
  for (int l = -1; l >= -5; l--) {
    if (!isNeighbor(column_indices, index + l, index + l + 1)) {
      break;
    }
    mask[index + l] = true;
  }
}

std::tuple<std::vector<float>, std::vector<int>>
calcCurvature(
  const pcl::PointCloud<pcl::PointXYZ> & points,
  const std::vector<float> & range,
  const int N_SCAN,
  const int horizontal_size)
{
  std::vector<float> curvature(N_SCAN * horizontal_size);
  std::vector<int> indices(N_SCAN * horizontal_size, -1);
  for (unsigned int i = 5; i < points.size() - 5; i++) {
    const float d =
      range[i - 5] + range[i - 4] + range[i - 3] + range[i - 2] + range[i - 1] -
      range[i] * 10 +
      range[i + 1] + range[i + 2] + range[i + 3] + range[i + 4] + range[i + 5];

    curvature[i] = d * d;
    indices[i] = i;
  }
  return {curvature, indices};
}

class IndexRange
{
public:
  IndexRange(const int start_index, const int end_index, const int n_blocks)
  : start_index_(static_cast<double>(start_index)),
    end_index_(static_cast<double>(end_index)),
    n_blocks_(static_cast<double>(n_blocks))
  {
  }

  int begin(const int j) const
  {
    const double n = n_blocks_;
    return static_cast<int>(start_index_ * (1. - j / n) + end_index_ * j / n);
  }

  int end(const int j) const
  {
    const double n = n_blocks_;
    const int k = j + 1;
    return static_cast<int>(start_index_ * (1. - k / n) + end_index_ * k / n - 1.);
  }

private:
  const double start_index_;
  const double end_index_;
  const double n_blocks_;
};

//  Lidar Sensor Configuration
const int N_SCAN = 16;
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

//  CPU Params
const int n_cores = 2;


class FeatureExtraction : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr edge_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_publisher_;

  std::deque<sensor_msgs::msg::PointCloud2> cloud_queue_;

public:
  FeatureExtraction()
  : Node("lidar_feature_extraction")
  {
    cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_raw", 5,
      std::bind(&FeatureExtraction::cloudHandler, this, std::placeholders::_1));
    edge_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_edge", 1);
    surface_publisher_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("scan_surface", 1);
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    RCLCPP_INFO(this->get_logger(), "Feature extraction node created");
  }

  ~FeatureExtraction() {}

  void cloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg)
  {
    cloud_queue_.push_back(*laserCloudMsg);
    if (cloud_queue_.size() <= 2) {
      return;
    }

    const sensor_msgs::msg::PointCloud2 cloud_msg = cloud_queue_.front();
    cloud_queue_.pop_front();

    const pcl::PointCloud<PointXYZIRT> input_points = *getPointCloud<PointXYZIRT>(cloud_msg);

    if (!input_points.is_dense) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Point cloud is not in dense format, please remove NaN points first!");
      rclcpp::shutdown();
    }

    if (!ringIsAvailable(cloud_msg)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Point cloud ring channel could not be found");
      rclcpp::shutdown();
    }

    if (!timeStampIsAvailable(cloud_msg)) {
      RCLCPP_ERROR(this->get_logger(), "Point cloud timestamp not available");
      rclcpp::shutdown();
    }

    const auto [indices, times, points] = extractElements(
      input_points, range_min, range_max, HORIZONTAL_SIZE
    );
    std::unordered_map<int, Eigen::Vector3d> output_points = projectWithoutImu(indices, points);

    std::vector<int> start_ring_indices(N_SCAN, 0);
    std::vector<int> end_ring_indices(N_SCAN, 0);

    std::vector<int> column_indices(N_SCAN * HORIZONTAL_SIZE, 0);
    std::vector<float> range(N_SCAN * HORIZONTAL_SIZE, 0);

    const auto range_map = makeRangeMatrix(indices, points);
    pcl::PointCloud<pcl::PointXYZ> cloud;

    int count = 0;
    for (int row_index = 0; row_index < N_SCAN; ++row_index) {
      start_ring_indices[row_index] = count + 5;

      for (int column_index = 0; column_index < HORIZONTAL_SIZE; ++column_index) {
        const int index = column_index + row_index * HORIZONTAL_SIZE;
        if (output_points.find(index) == output_points.end()) {
          continue;
        }

        column_indices[count] = column_index;
        range[count] = range_map.at(index);
        cloud.push_back(makePointXYZ(output_points[index]));
        count += 1;
      }

      end_ring_indices[row_index] = count - 5;
    }

    // used to prevent from labeling a neighbor as surface or edge
    std::vector<bool> mask(N_SCAN * HORIZONTAL_SIZE);

    for (unsigned int i = 5; i < cloud.size() - 5; i++) {
      mask[i] = false;
    }

    // mark occluded points and parallel beam points
    for (unsigned int i = 5; i < cloud.size() - 6; ++i) {
      if (!isNeighbor(column_indices, i + 1, i)) {
        continue;
      }

      if (range[i] > range[i + 1] + 0.3) {
        for (int j = 0; j <= 5; j++) {
          mask[i - j] = true;
        }
      }

      if (range[i + 1] > range[i] + 0.3) {
        for (int j = 1; j <= 6; j++) {
          mask[i + j] = true;
        }
      }
    }

    for (unsigned int i = 5; i < cloud.size() - 6; ++i) {
      // parallel beam
      const float ratio1 = std::abs(range[i - 1] - range[i]) / range[i];
      const float ratio2 = std::abs(range[i + 1] - range[i]) / range[i];

      if (ratio1 > 0.02 && ratio2 > 0.02) {
        mask[i] = true;
      }
    }

    auto [curvature, inds] = calcCurvature(cloud, range, N_SCAN, HORIZONTAL_SIZE);

    pcl::PointCloud<pcl::PointXYZ>::Ptr edge(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface(new pcl::PointCloud<pcl::PointXYZ>());

    const int N_BLOCKS = 6;

    std::vector<CurvatureLabel> label(N_SCAN * HORIZONTAL_SIZE, CurvatureLabel::Default);

    for (int i = 0; i < N_SCAN; i++) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr surface_scan(new pcl::PointCloud<pcl::PointXYZ>());

      const IndexRange index_range(start_ring_indices[i], end_ring_indices[i], N_BLOCKS);
      for (int j = 0; j < N_BLOCKS; j++) {
        const int sp = index_range.begin(j);
        const int ep = index_range.end(j);
        std::sort(inds.begin() + sp, inds.begin() + ep, by_value(curvature));

        int n_picked = 0;
        for (int k = ep; k >= sp; k--) {
          const int index = inds[k];
          if (mask[index] || curvature[index] <= edgeThreshold) {
            continue;
          }

          if (n_picked >= 20) {
            break;
          }

          n_picked++;

          edge->push_back(cloud.at(index));
          label[index] = CurvatureLabel::Edge;

          neighborPicked(column_indices, index, mask);
        }

        for (int k = sp; k <= ep; k++) {
          const int index = inds[k];
          if (mask[index] || curvature[index] >= surfThreshold) {
            continue;
          }

          label[index] = CurvatureLabel::Surface;

          neighborPicked(column_indices, index, mask);
        }

        for (int k = sp; k <= ep; k++) {
          if (label[k] == CurvatureLabel::Default || label[k] == CurvatureLabel::Edge) {
            surface_scan->push_back(cloud.at(k));
          }
        }
      }

      *surface += *downsample<pcl::PointXYZ>(surface_scan, surface_leaf_size);
    }

    const auto edge_downsampled = downsample<pcl::PointXYZ>(edge, map_edge_leaf_size);
    const auto surface_downsampled = downsample<pcl::PointXYZ>(surface, map_surface_leaf_size);

    const std::string lidar_frame = "base_link";
    const auto cloud_edge = toRosMsg(*edge_downsampled, cloud_msg.header.stamp, lidar_frame);
    const auto cloud_surface = toRosMsg(*surface_downsampled, cloud_msg.header.stamp, lidar_frame);
    edge_publisher_->publish(cloud_edge);
    surface_publisher_->publish(cloud_surface);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeatureExtraction>());
  rclcpp::shutdown();
  return 0;
}
