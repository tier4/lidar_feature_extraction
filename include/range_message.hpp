// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef RANGE_MESSAGE_HPP_
#define RANGE_MESSAGE_HPP_

#include <fmt/core.h>

#include <string>

template<typename T>
std::string RangeMessageLargerThanOrEqualTo(
  const std::string & value_name,
  const std::string & range_name,
  const T value,
  const T range_max)
{
  return fmt::format(
    "{} (which is {}) >= {} (which is {})",
    value_name, value, range_name, range_max);
}

template<typename T>
std::string RangeMessageSmallerThanOrEqualTo(
  const std::string & value_name,
  const std::string & range_name,
  const T value,
  const T range_max)
{
  return fmt::format(
    "{} (which is {}) <= {} (which is {})",
    value_name, value, range_name, range_max);
}

template<typename T>
std::string RangeMessageLargerThan(
  const std::string & value_name,
  const std::string & range_name,
  const T value,
  const T range_max)
{
  return fmt::format(
    "{} (which is {}) > {} (which is {})",
    value_name, value, range_name, range_max);
}

template<typename T>
std::string RangeMessageSmallerThan(
  const std::string & value_name,
  const std::string & range_name,
  const T value,
  const T range_max)
{
  return fmt::format(
    "{} (which is {}) < {} (which is {})",
    value_name, value, range_name, range_max);
}

#endif  // RANGE_MESSAGE_HPP_
