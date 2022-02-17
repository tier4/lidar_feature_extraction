// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <boost/range/adaptor/reversed.hpp>

#include <fmt/core.h>

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

#include <algorithm>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

struct PointXYZIR
{
  PCL_ADD_POINT4D PCL_ADD_INTENSITY;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
  PointXYZIR,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)
)

template<typename T>
pcl::PointXYZ MakePointXYZ(const T & input)
{
  return pcl::PointXYZ(input.x, input.y, input.z);
}

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

bool IsInInclusiveRange(const double v, const double min, const double max)
{
  return min <= v && v <= max;
}

template<typename T>
pcl::PointCloud<T> FilterByRange(
  const pcl::PointCloud<T> & points,
  const double range_min,
  const double range_max)
{
  pcl::PointCloud<T> result;
  for (const T & p : points) {
    const double norm = Eigen::Vector2d(p.x, p.y).norm();
    if (IsInInclusiveRange(norm, range_min, range_max)) {
      result.push_back(p);
    }
  }
  return result;
}

template<typename T>
std::unordered_map<int, T>
ExtractElements(
  const std::function<int(T)> & point_to_index,
  const pcl::PointCloud<T> & input_points)
{
  std::unordered_map<int, T> output_points;
  for (const T & p : input_points) {
    const int index = point_to_index(p);
    if (output_points.find(index) != output_points.end()) {
      continue;
    }

    output_points[index] = p;
  }

  return output_points;
}

template<typename T>
class by_value
{
public:
  explicit by_value(const std::vector<T> & values)
  : values_(values) {}
  bool operator()(const int & left, const int & right)
  {
    return values_.at(left) < values_.at(right);
  }

private:
  const std::vector<T> & values_;
};

enum class CurvatureLabel
{
  Default = 0,
  Edge = 1,
  Surface = 2,
  OutOfRange = 3
};

inline double XYNorm(const double x, const double y)
{
  return std::sqrt(x * x + y * y);
}

double CalcRadian(const double x1, const double y1, const double x2, const double y2)
{
  const double dot = x1 * x2 + y1 * y2;
  const double norm1 = XYNorm(x1, y1);
  const double norm2 = XYNorm(x2, y2);
  const double cos_angle = dot / (norm1 * norm2);
  return std::acos(cos_angle);
}

template<typename PointT>
bool IsNeighbor(const PointT & p1, const PointT & p2, const double radian_threshold)
{
  return CalcRadian(p1->x, p1->y, p2->x, p2->y) <= radian_threshold;
}

template<typename PointT>
using CloudIterator = typename pcl::PointCloud<PointT>::iterator;

template<typename PointT>
using CloudConstIterator = typename pcl::PointCloud<PointT>::const_iterator;

template<typename PointT>
pcl::PointCloud<PointT> ExtractEdge(
  const CloudConstIterator<PointT> cloud_begin,
  const std::vector<CurvatureLabel> & labels)
{
  typename pcl::PointCloud<PointT>::Ptr edge(new pcl::PointCloud<pcl::PointXYZ>());
  for (unsigned int i = 0; i < labels.size(); i++) {
    if (labels[i] == CurvatureLabel::Edge) {
      const PointT point = *(cloud_begin + i);
      edge->push_back(point);
    }
  }
  return edge;
}

template<typename PointT>
std::vector<std::pair<CloudConstIterator<PointT>, CloudConstIterator<PointT>>>
ExtractSectionsByRing(const typename pcl::PointCloud<PointT>::Ptr & cloud)
{
  const auto & points = cloud->points;

  if (points.size() == 0) {
    return {};
  }

  using T = CloudConstIterator<PointT>;

  const T cloud_begin = points.begin();

  std::set<std::uint16_t> rings;
  std::vector<std::pair<T, T>> sections;

  T begin = cloud_begin;
  std::uint16_t prev_ring = points.at(0).ring;

  for (unsigned int i = 1; i < points.size(); i++) {
    const T p = cloud_begin + i;
    if (p->ring == prev_ring) {
      continue;
    }

    if (rings.find(p->ring) != rings.end()) {
      auto s = fmt::format("Ring {} has already appeared", p->ring);
      throw std::invalid_argument(s);
    }

    rings.insert(p->ring);

    sections.push_back(std::make_pair(begin, p));

    begin = p;
    prev_ring = p->ring;
  }

  sections.push_back(std::make_pair(begin, points.end()));

  return sections;
}

template<typename PointT>
std::vector<double> CalcRange(
  const CloudConstIterator<PointT> cloud_begin,
  const CloudConstIterator<PointT> cloud_end)
{
  std::vector<double> range(cloud_end - cloud_begin);
  for (unsigned int i = 0; i < range.size(); i++) {
    const auto p = cloud_begin + i;
    range[i] = Eigen::Vector2d(p->x, p->y).norm();
  }
  return range;
}

template<typename T>
void FillFromLeft(
  std::vector<bool> & mask,
  const CloudConstIterator<T> & cloud_begin,
  const double radian_threshold,
  int begin_index,
  int end_index,
  const bool & value)
{
  for (int i = begin_index; i < end_index - 1; i++) {
    mask.at(i) = value;

    auto p0 = cloud_begin + i + 0;
    auto p1 = cloud_begin + i + 1;
    if (!IsNeighbor(p0, p1, radian_threshold)) {
      return;
    }
  }
  mask.at(end_index - 1) = value;
}

template<typename T>
void FillFromRight(
  std::vector<bool> & mask,
  const CloudConstIterator<T> & cloud_begin,
  const double radian_threshold,
  int begin_index,
  int end_index,
  const bool & value)
{
  for (int i = end_index - 1; i > begin_index; i--) {
    mask.at(i) = value;

    auto p0 = cloud_begin + i - 0;
    auto p1 = cloud_begin + i - 1;
    if (!IsNeighbor(p0, p1, radian_threshold)) {
      return;
    }
  }
  mask.at(begin_index) = value;
}

template<typename PointT>
void MaskOccludedPoints(
  std::vector<bool> & mask,
  const CloudConstIterator<PointT> & cloud_begin,
  const CloudConstIterator<PointT> & cloud_end,
  const int padding,
  const double distance_diff_threshold,
  const double radian_threshold)
{
  const int cloud_size = cloud_end - cloud_begin;
  assert(static_cast<std::uint32_t>(cloud_size) == mask.size());

  for (int i = 0; i < cloud_size - 1; i++) {
    const auto p0 = cloud_begin + i + 0;
    const auto p1 = cloud_begin + i + 1;

    if (!IsNeighbor(p0, p1, radian_threshold)) {
      continue;
    }

    const double range0 = XYNorm(p0->x, p0->y);
    const double range1 = XYNorm(p1->x, p1->y);

    if (range0 > range1 + distance_diff_threshold) {
      FillFromRight<PointT>(mask, cloud_begin, radian_threshold, i - padding, i + 1, true);
    }

    if (range1 > range0 + distance_diff_threshold) {
      FillFromLeft<PointT>(mask, cloud_begin, radian_threshold, i + 1, i + padding + 2, true);
    }
  }
}

template<typename PointT>
void MaskParallelBeamPoints(
  std::vector<bool> & mask,
  const CloudConstIterator<PointT> & cloud_begin,
  const CloudConstIterator<PointT> & cloud_end,
  const double range_ratio_threshold)
{
  const std::vector<double> ranges = CalcRange<PointT>(cloud_begin, cloud_end);
  for (unsigned int i = 1; i < ranges.size() - 1; ++i) {
    const float ratio1 = std::abs(ranges.at(i - 1) - ranges.at(i)) / ranges.at(i);
    const float ratio2 = std::abs(ranges.at(i + 1) - ranges.at(i)) / ranges.at(i);

    if (ratio1 > range_ratio_threshold && ratio2 > range_ratio_threshold) {
      mask.at(i) = true;
    }
  }
}

template<typename PointT>
void FillNeighbors(
  std::vector<bool> & mask,
  const CloudConstIterator<PointT> & cloud_begin,
  const int index,
  const int padding,
  const double radian_threshold)
{
  mask.at(index) = true;
  FillFromLeft<PointT>(mask, cloud_begin, radian_threshold, index + 1, index + 1 + padding, true);
  FillFromRight<PointT>(mask, cloud_begin, radian_threshold, index - padding, index, true);
}

template<typename T>
std::vector<int> SortedIndices(const std::vector<T> & values)
{
  // auto by_value = [&](const int left, const int right) {
  //   return values.at(left) < values.at(right);
  // };

  const int size = static_cast<int>(values.size());
  std::vector<int> indices = ranges::views::ints(0, size) | ranges::to_vector;
  std::sort(indices.begin(), indices.end(), by_value(values));
  return indices;
}

template<typename T>
std::string RangeMessageLargerOrEqualTo(
  const std::string & value_name,
  const std::string & range_name,
  const T value,
  const T range_max)
{
  return fmt::format(
    "{} (which is {}) >= {} (which is {})",
    value_name, value, range_name, range_max);
}

template<typename T>
std::string RangeMessageSmallerThan(
  const std::string & value_name,
  const std::string & range_name,
  const T value,
  const T range_max)
{
  return fmt::format(
    "{} (which is {}) < {} (which is {})",
    value_name, value, range_name, range_max);
}

class IndexRange
{
public:
  IndexRange(const int start_index, const int end_index, const int n_blocks)
  : start_index_(start_index), end_index_(end_index), n_blocks_(n_blocks)
  {
  }

  int Begin(const int j) const
  {
    ThrowExceptionIfOutOfRange(j);
    return this->Boundary(j);
  }

  int End(const int j) const
  {
    ThrowExceptionIfOutOfRange(j);
    return this->Boundary(j + 1);
  }

protected:
  int Boundary(const int j) const
  {
    const double s = static_cast<double>(start_index_);
    const double e = static_cast<double>(end_index_);
    const double n = static_cast<double>(n_blocks_);
    return static_cast<int>(s * (1. - j / n) + e * j / n);
  }

  void ThrowExceptionIfOutOfRange(const int j) const
  {
    if (j >= n_blocks_) {
      auto s = RangeMessageLargerOrEqualTo("j", "n_blocks", j, n_blocks_);
      throw std::out_of_range(s);
    }

    if (j < 0) {
      auto s = RangeMessageSmallerThan("j", "0", j, 0);
      throw std::out_of_range(s);
    }
  }

private:
  const int start_index_;
  const int end_index_;
  const int n_blocks_;
};

class PaddedIndexRange : public IndexRange
{
public:
  PaddedIndexRange(
    const int start_index, const int end_index, const int n_blocks,
    const int padding)
  : IndexRange(start_index + padding, end_index - padding, n_blocks), padding_(padding)
  {
  }

  int Begin(const int j) const
  {
    return IndexRange::Begin(j) - padding_;
  }

  int End(const int j) const
  {
    return IndexRange::End(j) + padding_;
  }

private:
  const int padding_;
};

template<typename T1, typename T2>
double InnerProduct(T1 first1, T1 last1, T2 first2)
{
  double sum = 0.;
  while (first1 != last1) {
    sum += (*first1) * (*first2);
    first1++;
    first2++;
  }
  return sum;
}

template<typename T1, typename T2>
std::vector<double> Convolution1D(
  const T1 input_begin, const T1 input_end,
  const T2 weight_begin, const T2 weight_end)
{
  const int input_size = input_end - input_begin;
  const int weight_size = weight_end - weight_begin;

  if (input_size < weight_size) {
    auto s = fmt::format(
      "Input array size {} cannot be smaller than weight size {}",
      input_size, weight_size);
    throw std::invalid_argument(s);
  }

  std::vector<double> result(input_size - weight_size + 1);
  for (unsigned int i = 0; i < result.size(); i++) {
    auto iter = input_begin + i;
    result[i] = InnerProduct(iter, iter + weight_size, weight_begin);
  }
  return result;
}

std::vector<double> CalcCurvature(const std::vector<double> & range)
{
  const std::vector<double> weight{1., 1., 1., 1., 1., -10., 1., 1., 1., 1., 1.};
  auto f = [](const double v) {return v * v;};
  const auto weighted = Convolution1D(range.begin(), range.end(), weight.begin(), weight.end());
  return weighted | ranges::views::transform(f) | ranges::to_vector;
}

template<typename PointT>
std::vector<CurvatureLabel>
AssignLabelToPoints(
  const CloudConstIterator<PointT> cloud_begin,
  const CloudConstIterator<PointT> cloud_end,
  const int n_blocks)
{
  const int cloud_size = cloud_end - cloud_begin;

  const int padding = 5;
  const double radian_threshold = 2.0;
  const double distance_diff_threshold = 0.3;
  const double range_ratio_threshold = 0.02;
  const double edge_threshold = 0.1;
  const double surface_threshold = 0.1;
  const PaddedIndexRange index_range(0, cloud_size, n_blocks, padding);

  std::vector<bool> mask(cloud_size, false);
  MaskOccludedPoints<PointT>(
    mask, cloud_begin, cloud_end, padding,
    distance_diff_threshold, radian_threshold
  );
  MaskParallelBeamPoints<PointT>(mask, cloud_begin, cloud_end, range_ratio_threshold);
  std::vector<CurvatureLabel> labels(cloud_size, CurvatureLabel::Default);

  for (int j = 0; j < n_blocks; j++) {
    const auto block_begin = cloud_begin + index_range.Begin(j);
    const auto block_end = cloud_begin + index_range.End(j);

    const std::vector<double> ranges = CalcRange<PointT>(block_begin, block_end);
    const std::vector<double> curvature = CalcCurvature(ranges);
    const std::vector<int> indices = SortedIndices(curvature);

    const int expected_size = index_range.End(j) - index_range.Begin(j) - 2 * padding;
    assert(curvature.size() == static_cast<std::uint32_t>(expected_size));

    const int offset = index_range.Begin(j) + padding;
    int n_picked = 0;
    for (const int index : boost::adaptors::reverse(indices)) {
      if (mask.at(offset + index) || curvature.at(index) <= edge_threshold) {
        continue;
      }

      if (n_picked >= 20) {
        break;
      }

      n_picked++;

      labels.at(offset + index) = CurvatureLabel::Edge;

      FillNeighbors<PointT>(mask, cloud_begin, offset + index, padding, radian_threshold);
    }

    for (const int index : indices) {
      if (mask.at(offset + index) || curvature.at(index) >= surface_threshold) {
        continue;
      }

      labels.at(offset + index) = CurvatureLabel::Surface;

      FillNeighbors<PointT>(mask, cloud_begin, offset + index, padding, radian_threshold);
    }
  }
  return labels;
}

#endif  // _UTILITY_LIDAR_ODOMETRY_H_
