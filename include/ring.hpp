// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _RING_LIDAR_ODOMETRY_H_
#define _RING_LIDAR_ODOMETRY_H_

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

#endif  /* _RING_LIDAR_ODOMETRY_H_ */
