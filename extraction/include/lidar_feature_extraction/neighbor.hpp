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


#ifndef NEIGHBOR_HPP_
#define NEIGHBOR_HPP_

#include "math.hpp"
#include "neighbor.hpp"
#include "mapped_points.hpp"

template<typename PointT>
bool IsNeighbor(const PointT & p1, const PointT & p2, const double radian_threshold)
{
  return CalcRadian(p1.x, p1.y, p2.x, p2.y) <= radian_threshold;
}

template<typename PointT>
class Neighbor
{
public:
  Neighbor(const MappedPoints<PointT> & ref_points, const double radian_threshold)
  : ref_points_(ref_points), radian_threshold_(radian_threshold)
  {
  }

  bool operator()(const int index1, const int index2) const
  {
    const PointT & p1 = ref_points_.at(index1);
    const PointT & p2 = ref_points_.at(index2);
    return IsNeighbor(p1, p2, radian_threshold_);
  }

private:
  const MappedPoints<PointT> ref_points_;
  const double radian_threshold_;
};

#endif  // NEIGHBOR_HPP_