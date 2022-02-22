// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef MAPPED_POINTS_HPP_
#define MAPPED_POINTS_HPP_

#include <functional>
#include <vector>

#include <pcl/point_cloud.h>

#include "iterator.hpp"

template<typename Element>
class MappedPoints
{
public:
  MappedPoints(const pcl::PointCloud<Element> & iter, const std::vector<int> & indices)
  : iter_(iter), indices_(indices)
  {
  }

  int size() const
  {
    return indices_.size();
  }

  Element at(const int index) const
  {
    return iter_.at(indices_.at(index));
  }

private:
  const pcl::PointCloud<Element> iter_;
  const std::vector<int> indices_;
};

#endif  // MAPPED_POINTS_HPP_
