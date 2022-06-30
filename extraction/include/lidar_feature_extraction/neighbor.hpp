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


#ifndef LIDAR_FEATURE_EXTRACTION__NEIGHBOR_HPP_
#define LIDAR_FEATURE_EXTRACTION__NEIGHBOR_HPP_

#include <fmt/core.h>

#include <vector>
#include <string>

#include "lidar_feature_extraction/math.hpp"
#include "lidar_feature_extraction/mapped_points.hpp"
#include "lidar_feature_extraction/range_message.hpp"

#include "lidar_feature_library/span.hpp"

template<typename PointT>
bool IsNeighborXY(const PointT & p1, const PointT & p2, const double radian_threshold)
{
  return CalcRadian(p1.x, p1.y, p2.x, p2.y) < radian_threshold;
}

class NeighborCheckBase
{
public:
  virtual bool operator()(const int index1, const int index2) const
  {
    return index1 == index2;
  }

  virtual int size() const
  {
    return -1;
  }
};

template<typename PointT>
class NeighborCheckXY : public NeighborCheckBase
{
public:
  NeighborCheckXY(const MappedPoints<PointT> & ref_points, const double radian_threshold)
  : ref_points_(ref_points), radian_threshold_(radian_threshold)
  {
    if (ref_points.size() < 2) {
      auto s = fmt::format(
        "The input point size (which is {}) cannot be smaller than 2", ref_points.size());
      throw std::invalid_argument(s);
    }
  }

  bool operator()(const int index1, const int index2) const override
  {
    CheckIndex("index1", index1);
    CheckIndex("index2", index2);

    const PointT & p1 = ref_points_.at(index1);
    const PointT & p2 = ref_points_.at(index2);
    return IsNeighborXY(p1, p2, radian_threshold_);
  }

  int size() const override
  {
    return ref_points_.size();
  }

  NeighborCheckXY Slice(const int begin, const int end) const
  {
    return NeighborCheckXY(ref_points_.Slice(begin, end), radian_threshold_);
  }

private:
  void CheckIndex(const std::string & name, const int index) const
  {
    if (index < 0) {
      throw std::out_of_range(RangeMessageSmallerThan(name, "0", index, 0));
    }

    if (index >= this->size()) {
      throw std::out_of_range(
              RangeMessageLargerThanOrEqualTo(name, "this->size()", index, this->size()));
    }
  }

  const MappedPoints<PointT> ref_points_;
  const double radian_threshold_;
};


class NeighborCheckDebug : public NeighborCheckBase
{
public:
  explicit NeighborCheckDebug(const std::vector<int> & values)
  : values_(values)
  {
  }

  bool operator()(const int index1, const int index2) const override
  {
    return values_.at(index1) == values_.at(index2);
  }

  int size() const override
  {
    return values_.size();
  }

private:
  const std::vector<int> values_;
};

#endif  // LIDAR_FEATURE_EXTRACTION__NEIGHBOR_HPP_
