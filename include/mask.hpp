// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _MASK_LIDAR_ODOMETRY_H_
#define _MASK_LIDAR_ODOMETRY_H_

#include <string>
#include <vector>

#include "cloud_iterator.hpp"
#include "neighbor.hpp"
#include "range.hpp"
#include "reference_wrapper.hpp"

template<typename Element>
class Mask
{
public:
  Mask(
    const ConstReferenceVector<Element> & ref_points,
    const double radian_threshold)
  : mask_(std::vector<bool>(ref_points.size(), false)),
    ref_points_(ref_points),
    radian_threshold_(radian_threshold)
  {
  }

  Mask(const Mask & mask)
  : mask_(mask.mask_),
    ref_points_(mask.ref_points_),
    radian_threshold_(mask.radian_threshold_)
  {
  }

  void Fill(const int index)
  {
    mask_.at(index) = true;
  }

  void FillFromLeft(const int begin_index, const int end_index)
  {
    for (int i = begin_index; i < end_index - 1; i++) {
      mask_.at(i) = true;

      const Element & p0 = ref_points_.at(i + 0).get();
      const Element & p1 = ref_points_.at(i + 1).get();
      if (!IsNeighbor(p0, p1, radian_threshold_)) {
        return;
      }
    }
    mask_.at(end_index - 1) = true;
  }

  void FillFromRight(const int begin_index, const int end_index)
  {
    for (int i = end_index - 1; i > begin_index; i--) {
      mask_.at(i) = true;

      const Element & p0 = ref_points_.at(i - 0).get();
      const Element & p1 = ref_points_.at(i - 1).get();
      if (!IsNeighbor(p0, p1, radian_threshold_)) {
        return;
      }
    }
    mask_.at(begin_index) = true;
  }

  void FillNeighbors(const int index, const int padding)
  {
    this->Fill(index);
    this->FillFromLeft(index + 1, index + 1 + padding);
    this->FillFromRight(index - padding, index);
  }

  bool At(const int index) const
  {
    return mask_.at(index);
  }

  std::vector<bool> Get() const
  {
    return mask_;
  }

  int Size() const
  {
    return mask_.size();
  }

protected:
  void ThrowExceptionIfOutOfRange(
    const std::string & variable_name, const int variable) const
  {
    if (variable >= this->Size()) {
      auto s = RangeMessageLargerOrEqualTo(variable_name, "this->Size()", variable, this->Size());
      throw std::out_of_range(s);
    }

    if (variable < 0) {
      auto s = RangeMessageSmallerThan(variable_name, "0", variable, 0);
      throw std::out_of_range(s);
    }
  }

private:
  std::vector<bool> mask_;
  const ConstReferenceVector<Element> ref_points_;
  const double radian_threshold_;
};

template<typename PointT>
void MaskOccludedPoints(
  Mask<PointT> & mask,
  const Neighbor<PointT> & is_neighbor,
  const Range<PointT> & range,
  const int padding,
  const double distance_diff_threshold)
{
  for (int i = 0; i < mask.Size() - 1; i++) {
    if (!is_neighbor(i + 0, i + 1)) {
      continue;
    }

    const double range0 = range(i + 0);
    const double range1 = range(i + 1);

    if (range0 > range1 + distance_diff_threshold) {
      mask.FillFromRight(i - padding, i + 1);
    }

    if (range1 > range0 + distance_diff_threshold) {
      mask.FillFromLeft(i + 1, i + padding + 2);
    }
  }
}

template<typename PointT>
void MaskParallelBeamPoints(
  Mask<PointT> & mask,
  const Range<PointT> & range,
  const double range_ratio_threshold)
{
  const std::vector<double> ranges = range(0, mask.Size());
  for (int i = 1; i < mask.Size() - 1; ++i) {
    const float ratio1 = std::abs(ranges.at(i - 1) - ranges.at(i)) / ranges.at(i);
    const float ratio2 = std::abs(ranges.at(i + 1) - ranges.at(i)) / ranges.at(i);

    if (ratio1 > range_ratio_threshold && ratio2 > range_ratio_threshold) {
      mask.Fill(i);
    }
  }
}

#endif  /* _MASK_LIDAR_ODOMETRY_H_ */
