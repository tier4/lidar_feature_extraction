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
#include <fmt/core.h>


template<typename Iterator>
using ValueType = typename std::iterator_traits<Iterator>::value_type;


template<typename Iterator>
class Span
{
public:
  Span(Iterator begin, Iterator end)
  : begin_(begin), end_(end)
  {
  }

  Iterator Begin() const
  {
    return begin_;
  }

  Iterator End() const
  {
    return end_;
  }

  typename Iterator::reference At(const int i)
  {
    CheckIndex(i);
    return begin_[i];
  }

  typename Iterator::reference At(const int i) const
  {
    CheckIndex(i);
    return begin_[i];
  }

  void CheckIndex(const int i)
  {
    if (i < 0) {
      throw std::out_of_range(fmt::format("Index out of range. {} < 0", i));
    }

    if (i >= this->Size()) {
      throw std::out_of_range(fmt::format("Index out of range. {} >= this->Size()", i));
    }
  }

  int Size() const
  {
    return end_ - begin_;
  }

private:
  Iterator begin_;
  Iterator end_;
};

#endif  // SPAN_HPP_
