// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _RANGE_MESSAGE_LIDAR_ODOMETRY_H_
#define _RANGE_MESSAGE_LIDAR_ODOMETRY_H_

#include <string>

template<typename T>
std::string RangeMessageLargerOrEqualTo(
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

#endif  /* _RANGE_MESSAGE_LIDAR_ODOMETRY_H_ */
