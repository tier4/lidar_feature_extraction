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


#ifndef LIDAR_FEATURE_EXTRACTION__MAPPED_POINTS_HPP_
#define LIDAR_FEATURE_EXTRACTION__MAPPED_POINTS_HPP_

#include <pcl/point_cloud.h>

#include <functional>
#include <vector>

#include "iterator.hpp"

template<typename PointT>
class MappedPoints
{
public:
  MappedPoints(
    const typename pcl::PointCloud<PointT>::Ptr & cloud,
    const std::vector<int> & indices)
  : cloud_(cloud), indices_(indices)
  {
  }

  int size() const
  {
    return indices_.size();
  }

  PointT at(const int index) const
  {
    return cloud_->at(indices_.at(index));
  }

  MappedPoints Slice(const int begin, const int end) const
  {
    const std::vector<int> indices(indices_.begin() + begin, indices_.begin() + end);
    return MappedPoints<PointT>(cloud_, indices);
  }

private:
  const typename pcl::PointCloud<PointT>::Ptr cloud_;
  const std::vector<int> indices_;
};

#endif  // LIDAR_FEATURE_EXTRACTION__MAPPED_POINTS_HPP_
