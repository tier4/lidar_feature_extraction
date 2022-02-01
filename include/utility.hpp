#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>

#include <range/v3/all.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

struct PointXYZIR
{
  PCL_ADD_POINT4D PCL_ADD_INTENSITY;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIR,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (std::uint16_t, ring, ring)
)

template<typename T>
sensor_msgs::msg::PointCloud2 toRosMsg(const pcl::PointCloud<T> & pointcloud)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pointcloud, msg);
  return msg;
}

template<typename T>
sensor_msgs::msg::PointCloud2 toRosMsg(
  const pcl::PointCloud<T> & pointcloud,
  const rclcpp::Time stamp,
  const std::string frame)
{
  sensor_msgs::msg::PointCloud2 msg = toRosMsg(pointcloud);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame;
  return msg;
}

inline pcl::PointXYZ MakePointXYZ(const Eigen::Vector3d & v)
{
  return pcl::PointXYZ(v(0), v(1), v(2));
}

template<typename T>
typename pcl::PointCloud<T>::Ptr getPointCloud(const sensor_msgs::msg::PointCloud2 & roscloud)
{
  typename pcl::PointCloud<T>::Ptr pclcloud(new pcl::PointCloud<T>());
  pcl::fromROSMsg(roscloud, *pclcloud);
  return pclcloud;
}

template<typename T>
typename pcl::PointCloud<T>::Ptr downsample(
  const typename pcl::PointCloud<T>::Ptr & input_cloud, const float leaf_size)
{
  pcl::VoxelGrid<T> filter;
  typename pcl::PointCloud<T>::Ptr downsampled(new pcl::PointCloud<T>());

  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.setInputCloud(input_cloud);
  filter.filter(*downsampled);

  return downsampled;
}

bool RingIsAvailable(const std::vector<sensor_msgs::msg::PointField> & fields)
{
  for (const auto & field : fields) {
    if (field.name == "ring") {
      return true;
    }
  }
  return false;
}

int ColumnIndex(const int horizontal_size, const double x, const double y)
{
  const double k = horizontal_size * atan2(y, x) / (M_PI * 2.0);
  const double u = k + horizontal_size / 2.0;
  return static_cast<int>(u);
}

std::tuple<std::vector<int>, std::vector<Eigen::Vector3d>>
ExtractElements(
  const pcl::PointCloud<PointXYZIR> & input_points,
  const float range_min, const float range_max,
  const int horizontal_size)
{
  const auto f = [&](const PointXYZIR & p) {
      const int row_index = p.ring;
      const int column_index = ColumnIndex(horizontal_size, p.x, p.y);
      const int index = column_index + row_index * horizontal_size;
      const Eigen::Vector3d q(p.x, p.y, p.z);
      return std::make_tuple(index, q);
    };

  std::set<int> unique_indices;
  std::vector<int> indices;
  std::vector<Eigen::Vector3d> points;

  const auto iterator = input_points | ranges::views::transform(f);
  for (const auto & [index, point] : iterator) {
    const double range = point.norm();
    if (range < range_min || range_max < range) {
      continue;
    }

    if (unique_indices.find(index) != unique_indices.end()) {
      continue;
    }

    unique_indices.insert(index);
    indices.push_back(index);
    points.push_back(point);
  }

  return {indices, points};
}

std::unordered_map<int, Eigen::Vector3d> Projection(
  const std::vector<int> & indices,
  const std::vector<Eigen::Vector3d> points)
{
  std::unordered_map<int, Eigen::Vector3d> output_points;
  for (const auto & [index, q] : ranges::views::zip(indices, points)) {
    output_points.at(index) = q;
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
    return values_.at(left) < values_.at(right);
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

bool IsNeighbor(const std::vector<int> & column_indices, const int index1, const int index2)
{
  return std::abs(column_indices.at(index1) - column_indices.at(index2)) <= 10;
}

void NeighborPicked(
  const std::vector<int> & column_indices,
  const int index,
  std::vector<bool> & mask)
{
  mask.at(index) = true;
  for (int l = 1; l <= 5; l++) {
    if (!IsNeighbor(column_indices, index + l, index + l - 1)) {
      break;
    }
    mask.at(index + l) = true;
  }
  for (int l = -1; l >= -5; l--) {
    if (!IsNeighbor(column_indices, index + l, index + l + 1)) {
      break;
    }
    mask.at(index + l) = true;
  }
}

std::tuple<std::vector<float>, std::vector<int>>
CalcCurvature(
  const pcl::PointCloud<pcl::PointXYZ> & points,
  const std::vector<float> & range,
  const int N_SCAN,
  const int horizontal_size)
{
  std::vector<float> curvature(N_SCAN * horizontal_size);
  std::vector<int> indices(N_SCAN * horizontal_size, -1);
  for (unsigned int i = 5; i < points.size() - 5; i++) {
    const float d =
      range.at(i - 5) + range.at(i - 4) + range.at(i - 3) + range.at(i - 2) + range.at(i - 1) -
      range.at(i) * 10 +
      range.at(i + 1) + range.at(i + 2) + range.at(i + 3) + range.at(i + 4) + range.at(i + 5);

    curvature.at(i) = d * d;
    indices.at(i) = i;
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

  int Begin(const int j) const
  {
    const double n = n_blocks_;
    return static_cast<int>(start_index_ * (1. - j / n) + end_index_ * j / n);
  }

  int End(const int j) const
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

#endif  // _UTILITY_LIDAR_ODOMETRY_H_
