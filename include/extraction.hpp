// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef EXTRACTION_HPP_
#define EXTRACTION_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

#include "mapped_points.hpp"

template<typename PointT>
void ExtractByLabel(
  typename pcl::PointCloud<PointT>::Ptr output_cloud,
  const MappedPoints<PointT> & ref_points,
  const std::vector<CurvatureLabel> & labels,
  const CurvatureLabel & label)
{
  assert(ref_points.size() == labels.size());

  for (unsigned int i = 0; i < labels.size(); i++) {
    if (labels.at(i) == label) {
      output_cloud->push_back(ref_points.at(i));
    }
  }
}

#endif  // EXTRACTION_HPP_
