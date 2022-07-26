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


#include "lidar_feature_extraction/index_range.hpp"

IndexRange::IndexRange(const int start_index, const int end_index, const int n_blocks)
: start_index_(start_index), end_index_(end_index), n_blocks_(n_blocks)
{
  if (end_index - start_index < n_blocks) {
    auto s = fmt::format(
      "end_index - start_index (which is {}) cannot be smaller than n_blocks (which is {}",
      end_index - start_index, n_blocks);
    throw std::invalid_argument(s);
  }
}

int IndexRange::NBlocks() const
{
  return n_blocks_;
}

int IndexRange::Begin(const int j) const
{
  ThrowExceptionIfOutOfRange(j);
  return this->Boundary(j);
}

int IndexRange::End(const int j) const
{
  ThrowExceptionIfOutOfRange(j);
  return this->Boundary(j + 1);
}

int IndexRange::Boundary(const int j) const
{
  const double s = static_cast<double>(start_index_);
  const double e = static_cast<double>(end_index_);
  const double n = static_cast<double>(n_blocks_);
  return static_cast<int>(s * (1. - j / n) + e * j / n);
}

void IndexRange::ThrowExceptionIfOutOfRange(const int j) const
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
