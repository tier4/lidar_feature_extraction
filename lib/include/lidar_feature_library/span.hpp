// Copyright 2022 Takeshi Ishita
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
//    * Neither the name of the Takeshi Ishita nor the names of its
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


#ifndef SPAN_HPP_
#define SPAN_HPP_

#include <iterator>
#include <vector>

#include <fmt/core.h>


template<typename T>
class span
{
public:
  span(typename std::vector<T>::iterator begin, typename std::vector<T>::iterator end)
  : begin_(begin), end_(end)
  {
  }

  typename std::vector<T>::iterator begin() const
  {
    return begin_;
  }

  typename std::vector<T>::iterator end() const
  {
    return end_;
  }

  T & at(const int i)
  {
    check_index(i);
    return begin_[i];
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

private:
  void check_index(const int i)
  {
    if (i < 0) {
      throw std::out_of_range(fmt::format("Index out of range. {} < 0", i));
    }

    if (i >= this->size()) {
      throw std::out_of_range(fmt::format("Index out of range. {} >= this->size()", i));
    }
  }

  typename std::vector<T>::iterator begin_;
  typename std::vector<T>::iterator end_;
};

#endif  // SPAN_HPP_
