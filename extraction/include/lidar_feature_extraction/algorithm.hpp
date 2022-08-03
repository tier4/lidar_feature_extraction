// Copyright 2022 Tixiao Shan, Takeshi Ishita
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Tixiao Shan, Takeshi Ishita nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#ifndef LIDAR_FEATURE_EXTRACTION__ALGORITHM_HPP_
#define LIDAR_FEATURE_EXTRACTION__ALGORITHM_HPP_

#include <algorithm>
#include <vector>
#include <iterator>

#include <range/v3/all.hpp>

#include "iterator.hpp"

template<typename Iterator>
using ValueType = typename std::iterator_traits<Iterator>::value_type;

template<typename Iterator, typename T = ValueType<Iterator>>
class ByValue
{
public:
  explicit ByValue(const Iterator & values_begin)
  : begin_(values_begin) {}

  T at(const int i) const
  {
    return *(begin_ + i);
  }

  bool operator()(const int & left, const int & right)
  {
    return this->at(left) < this->at(right);
  }

private:
  const typename std::vector<T>::const_iterator & begin_;
};

template<typename Iterator, typename T = ValueType<Iterator>>
std::vector<int> Argsort(const Iterator & values_begin, const Iterator & values_end)
{
  std::vector<int> indices = irange(values_end - values_begin);
  std::sort(indices.begin(), indices.end(), ByValue(values_begin));
  return indices;
}

#endif  // LIDAR_FEATURE_EXTRACTION__ALGORITHM_HPP_
