// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef ALGORITHM_HPP_
#define ALGORITHM_HPP_

#include <algorithm>
#include <vector>

template<typename T>
class by_value
{
public:
  explicit by_value(const std::vector<T> & values)
  : values_(values) {}
  bool operator()(const int & left, const int & right)
  {
    return values_.at(left) < values_.at(right);
  }

private:
  const std::vector<T> & values_;
};

template<typename T>
std::vector<int> Argsort(const std::vector<T> & values)
{
  const int size = static_cast<int>(values.size());
  std::vector<int> indices = ranges::views::ints(0, size) | ranges::to_vector;
  std::sort(indices.begin(), indices.end(), by_value(values));
  return indices;
}

#endif  // ALGORITHM_HPP_
