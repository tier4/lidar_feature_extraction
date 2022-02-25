// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef INDEX_RANGE_HPP_
#define INDEX_RANGE_HPP_

#include <fmt/core.h>

#include "range_message.hpp"

class IndexRange
{
public:
  IndexRange(const int start_index, const int end_index, const int n_blocks)
  : start_index_(start_index), end_index_(end_index), n_blocks_(n_blocks)
  {
    if (end_index - start_index < n_blocks) {
      auto s = fmt::format(
        "end_index - start_index (which is {}) cannot be smaller than n_blocks (which is {}",
        end_index - start_index, n_blocks);
      throw std::invalid_argument(s);
    }
  }

  int Begin(const int j) const
  {
    ThrowExceptionIfOutOfRange(j);
    return this->Boundary(j);
  }

  int End(const int j) const
  {
    ThrowExceptionIfOutOfRange(j);
    return this->Boundary(j + 1);
  }

protected:
  int Boundary(const int j) const
  {
    const double s = static_cast<double>(start_index_);
    const double e = static_cast<double>(end_index_);
    const double n = static_cast<double>(n_blocks_);
    return static_cast<int>(s * (1. - j / n) + e * j / n);
  }

  void ThrowExceptionIfOutOfRange(const int j) const
  {
    if (j >= n_blocks_) {
      auto s = RangeMessageLargerThanOrEqualTo("j", "n_blocks", j, n_blocks_);
      throw std::out_of_range(s);
    }

    if (j < 0) {
      auto s = RangeMessageSmallerThan("j", "0", j, 0);
      throw std::out_of_range(s);
    }
  }

private:
  const int start_index_;
  const int end_index_;
  const int n_blocks_;
};

class PaddedIndexRange : public IndexRange
{
public:
  PaddedIndexRange(
    const int start_index, const int end_index, const int n_blocks,
    const int padding)
  : IndexRange(start_index + padding, end_index - padding, n_blocks), padding_(padding)
  {
  }

  int Begin(const int j) const
  {
    return IndexRange::Begin(j) - padding_;
  }

  int End(const int j) const
  {
    return IndexRange::End(j) + padding_;
  }

private:
  const int padding_;
};

#endif  // INDEX_RANGE_HPP_
