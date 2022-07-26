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

#ifndef LIDAR_FEATURE_EXTRACTION__COLOR_POINTS_HPP_
#define LIDAR_FEATURE_EXTRACTION__COLOR_POINTS_HPP_

#include <fmt/format.h>
#include <pcl/point_types.h>

#include <vector>

#include "lidar_feature_extraction/label.hpp"
#include "lidar_feature_extraction/point_label.hpp"


std::vector<uint8_t> LabelToColor(const PointLabel & label);

std::vector<uint8_t> ValueToColor(const double value, const double min, const double max);

template<typename PointT>
pcl::PointXYZRGB MakeXYZRGB(const PointT & p, const std::vector<uint8_t> & rgb)
{
  pcl::PointXYZRGB xyzrgb;

  xyzrgb.x = p.x;
  xyzrgb.y = p.y;
  xyzrgb.z = p.z;
  xyzrgb.r = rgb.at(0);
  xyzrgb.g = rgb.at(1);
  xyzrgb.b = rgb.at(2);

  return xyzrgb;
}

template<typename PointT>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorPointsByLabel(
  const MappedPoints<PointT> & ref_points,
  const std::vector<PointLabel> & labels)
{
  assert(ref_points.size() == static_cast<int>(labels.size()));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>());
  for (unsigned int i = 0; i < labels.size(); i++) {
    const PointT & p = ref_points.at(i);
    const std::vector<uint8_t> rgb = LabelToColor(labels.at(i));
    colored->push_back(MakeXYZRGB(p, rgb));
  }
  return colored;
}

template<typename PointT>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorPointsByValue(
  const MappedPoints<PointT> & ref_points,
  const std::vector<double> & values,
  const double min,
  const double max)
{
  assert(ref_points.size() == static_cast<int>(values.size()));
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored(new pcl::PointCloud<pcl::PointXYZRGB>());

  for (unsigned int i = 0; i < values.size(); i++) {
    const PointT & p = ref_points.at(i);
    const std::vector<uint8_t> rgb = ValueToColor(values.at(i), min, max);
    colored->push_back(MakeXYZRGB(p, rgb));
  }
  return colored;
}

#endif  // LIDAR_FEATURE_EXTRACTION__COLOR_POINTS_HPP_
