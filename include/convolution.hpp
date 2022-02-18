// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef CONVOLUTION_HPP_
#define CONVOLUTION_HPP_

#include <fmt/core.h>

#include <vector>

#include "math.hpp"

template<typename T1, typename T2>
std::vector<double> Convolution1D(
  const T1 input_begin, const T1 input_end,
  const T2 weight_begin, const T2 weight_end)
{
  const int input_size = input_end - input_begin;
  const int weight_size = weight_end - weight_begin;

  if (input_size < weight_size) {
    auto s = fmt::format(
      "Input array size {} cannot be smaller than weight size {}",
      input_size, weight_size);
    throw std::invalid_argument(s);
  }

  std::vector<double> result(input_size - weight_size + 1);
  for (unsigned int i = 0; i < result.size(); i++) {
    auto iter = input_begin + i;
    result[i] = InnerProduct(iter, iter + weight_size, weight_begin);
  }
  return result;
}

#endif  // CONVOLUTION_HPP_
