// Copyright 2022 Tixiao Shan, Takeshi Ishita (2020)
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
//    * Neither the name of the Tixiao Shan, Takeshi Ishita (2020) nor the names of its
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


#ifndef MASK_HPP_
#define MASK_HPP_

#include <string>
#include <vector>

#include "cloud_iterator.hpp"
#include "neighbor.hpp"
#include "range.hpp"
#include "range_message.hpp"
#include "mapped_points.hpp"

template<typename Element>
class Mask
{
public:
  Mask(
    const MappedPoints<Element> & ref_points,
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
    if (end_index > this->Size()) {
      auto s = RangeMessageLargerThan(
        "end_index", "this->Size()", end_index, this->Size());
      throw std::invalid_argument(s);
    }

    if (begin_index < 0) {
      auto s = RangeMessageSmallerThan("begin_index", "0", begin_index, 0);
      throw std::invalid_argument(s);
    }

    for (int i = begin_index; i < end_index - 1; i++) {
      mask_.at(i) = true;

      const Element & p0 = ref_points_.at(i + 0);
      const Element & p1 = ref_points_.at(i + 1);
      if (!IsNeighbor(p0, p1, radian_threshold_)) {
        return;
      }
    }
    mask_.at(end_index - 1) = true;
  }

  void FillFromRight(const int begin_index, const int end_index)
  {
    if (end_index >= this->Size()) {
      auto s = RangeMessageLargerThanOrEqualTo(
        "end_index", "this->Size()", end_index, this->Size());
      throw std::invalid_argument(s);
    }

    if (begin_index < -1) {
      auto s = RangeMessageSmallerThan("begin_index", "-1", begin_index, -1);
      throw std::invalid_argument(s);
    }

    for (int i = end_index; i > begin_index + 1; i--) {
      mask_.at(i) = true;

      const Element & p0 = ref_points_.at(i - 0);
      const Element & p1 = ref_points_.at(i - 1);
      if (!IsNeighbor(p0, p1, radian_threshold_)) {
        return;
      }
    }
    mask_.at(begin_index + 1) = true;
  }

  void FillNeighbors(const int index, const int padding)
  {
    if (index + padding >= this->Size()) {
      auto s = RangeMessageLargerThanOrEqualTo(
        "index + padding", "this->Size()", index + padding, this->Size());
      throw std::invalid_argument(s);
    }

    if (index - padding < 0) {
      auto s = RangeMessageSmallerThan(
        "index - padding", "0", index - padding, 0);
      throw std::invalid_argument(s);
    }

    this->Fill(index);
    this->FillFromLeft(index + 1, index + 1 + padding);
    this->FillFromRight(index - padding - 1, index - 1);
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

private:
  std::vector<bool> mask_;
  const MappedPoints<Element> ref_points_;
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
  for (int i = padding; i < mask.Size() - padding - 1; i++) {
    if (!is_neighbor(i + 0, i + 1)) {
      continue;
    }

    const double range0 = range(i + 0);
    const double range1 = range(i + 1);

    if (range0 > range1 + distance_diff_threshold) {
      mask.FillFromRight(i - padding - 1, i);
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
  for (int i = 1; i < mask.Size() - 1; i++) {
    const float ratio1 = std::abs(ranges.at(i - 1) - ranges.at(i)) / ranges.at(i);
    const float ratio2 = std::abs(ranges.at(i + 1) - ranges.at(i)) / ranges.at(i);

    if (ratio1 > range_ratio_threshold && ratio2 > range_ratio_threshold) {
      mask.Fill(i);
    }
  }
}

#endif  // MASK_HPP_
