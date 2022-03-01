// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef ITERATOR_HPP_
#define ITERATOR_HPP_

#include <range/v3/all.hpp>

#include <vector>

template<typename Iterator>
using ElementType = typename std::iterator_traits<typename Iterator::iterator>::value_type;

std::vector<int> irange(const int size)
{
  return ranges::views::ints(0, size) | ranges::to_vector;
}

#endif  // ITERATOR_HPP_
