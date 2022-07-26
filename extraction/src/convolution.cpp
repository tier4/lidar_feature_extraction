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

#include <cassert>
#include <vector>

#include "lidar_feature_extraction/convolution.hpp"


std::vector<double> Convolution1D(
  const std::vector<double> input,
  const std::vector<double> weight)
{
  if (input.size() < weight.size()) {
    auto s = fmt::format(
      "Input array size {} cannot be smaller than weight size {}", input.size(), weight.size());
    throw std::invalid_argument(s);
  }

  assert(weight.size() % 2 == 1);

  const int padding = (weight.size() - 1) / 2;
  const int convolution_size = input.size() - padding * 2;

  std::vector<double> result(input.size());

  for (int i = 0; i < padding; i++) {
    result[i] = 0.;
  }

  for (int i = 0; i < convolution_size; i++) {
    const auto iter = input.begin() + i;
    result[padding + i] = InnerProduct(iter, iter + weight.size(), weight.begin());
  }

  for (int i = 0; i < padding; i++) {
    result[convolution_size + padding + i] = 0.;
  }

  return result;
}
