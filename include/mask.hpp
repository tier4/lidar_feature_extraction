// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include "neighbor.hpp"
#include "cloud_iterator.hpp"

template<typename PointT>
class Mask
{
public:
  Mask(
    const CloudConstIterator<PointT> & cloud_begin,
    const CloudConstIterator<PointT> & cloud_end,
    const double radian_threshold)
  : mask_(std::vector<bool>(cloud_end - cloud_begin, false)),
    cloud_begin_(cloud_begin),
    radian_threshold_(radian_threshold)
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

      auto p0 = cloud_begin_ + i + 0;
      auto p1 = cloud_begin_ + i + 1;
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

      auto p0 = cloud_begin_ + i - 0;
      auto p1 = cloud_begin_ + i - 1;
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

private:
  std::vector<bool> mask_;
  const CloudConstIterator<PointT> cloud_begin_;
  const double radian_threshold_;
};