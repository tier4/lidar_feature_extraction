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


#ifndef LIDAR_FEATURE_LIBRARY__SPAN_HPP_
#define LIDAR_FEATURE_LIBRARY__SPAN_HPP_

#include <fmt/core.h>

#include <iterator>
#include <vector>


template<typename T, typename Iterator>
class span_base
{
public:
  Iterator begin() const
  {
    return begin_;
  }

  Iterator end() const
  {
    return end_;
  }

  T at(const int i) const
  {
    check_index(i);
    return begin_[i];
  }

  int size() const
  {
    return end_ - begin_;
  }

protected:
  span_base(Iterator begin, Iterator end)
  : begin_(begin), end_(end)
  {
  }

  void check_index(const int i) const
  {
    if (i < 0) {
      throw std::out_of_range(fmt::format("Index out of range. {} < 0", i));
    }

    if (i >= this->size()) {
      throw std::out_of_range(fmt::format("Index out of range. {} >= this->size()", i));
    }
  }

  Iterator begin_;
  Iterator end_;
};

template<typename T>
class const_span : public span_base<T, typename std::vector<T>::const_iterator>
{
public:
  const_span(
    typename std::vector<T>::const_iterator begin,
    typename std::vector<T>::const_iterator end)
  : span_base<T, typename std::vector<T>::const_iterator>(begin, end)
  {
  }
};

template<typename T>
class span : public span_base<T, typename std::vector<T>::iterator>
{
public:
  span(
    typename std::vector<T>::iterator begin,
    typename std::vector<T>::iterator end)
  : span_base<T, typename std::vector<T>::iterator>(begin, end)
  {
  }

  T & at(const int i)
  {
    this->check_index(i);
    return this->begin_[i];
  }
};

#endif  // LIDAR_FEATURE_LIBRARY__SPAN_HPP_
