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


#ifndef RANGE_HPP_
#define RANGE_HPP_

#include <vector>

#include "cloud_iterator.hpp"
#include "iterator.hpp"
#include "math.hpp"
#include "mapped_points.hpp"

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
    const double norm = XYNorm(p.x, p.y);
    if (IsInInclusiveRange(norm, range_min, range_max)) {
      result.push_back(p);
    }
  }
  return result;
}

template<typename PointT>
class Range
{
public:
  explicit Range(const MappedPoints<PointT> & ref_points)
  : ref_points_(ref_points) {}

  double operator()(const int index) const
  {
    const PointT & p = ref_points_.at(index);
    return XYNorm(p.x, p.y);
  }

  std::vector<double> operator()(const int begin, const int end) const
  {
    std::vector<double> ranges(end - begin);
    for (unsigned int i = 0; i < ranges.size(); i++) {
      ranges.at(i) = (*this)(i);
    }
    return ranges;
  }

  int Size() const
  {
    return ref_points_.size();
  }

private:
  const MappedPoints<PointT> ref_points_;
};

#endif  // RANGE_HPP_
