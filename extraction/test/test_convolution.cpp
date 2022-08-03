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


#include <gmock/gmock.h>

#include <vector>

#include "lidar_feature_extraction/convolution.hpp"


TEST(Convolution, Convolution1D) {
  {
    std::vector<double> input{1., -1, 2., 0., 1};
    std::vector<double> weight{1., 0., -1};

    const auto result = Convolution1D(input, weight);

    EXPECT_THAT(result.size(), input.size());
    EXPECT_THAT(result, testing::ElementsAre(0., -1., -1., 1., 0.));
  }

  {
    std::vector<double> input{1., -1, 2.};
    std::vector<double> weight{1., 0., -1};

    const auto result = Convolution1D(input, weight);

    EXPECT_THAT(result.size(), input.size());
    EXPECT_THAT(result, testing::ElementsAre(0., -1., 0.));
  }

  {
    std::vector<double> input{2., 0.};
    std::vector<double> weight{1., 0., -1};
    EXPECT_THROW(
      try {
      Convolution1D(input, weight);
    } catch (const std::invalid_argument & e) {
      EXPECT_STREQ("Input array size 2 cannot be smaller than weight size 3", e.what());
      throw e;
    }
      ,
      std::invalid_argument);
  }
}
