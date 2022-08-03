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

#include "lidar_feature_library/span.hpp"


TEST(Span, NonConstSpan)
{
  std::vector<int> v{0, 1, 2, 3, 4, 5, 6};
  span<int> span(v.begin(), v.end());

  EXPECT_EQ(span.size(), 7);

  EXPECT_EQ(span.at(0), 0);
  EXPECT_EQ(span.at(4), 4);

  EXPECT_EQ(span.begin(), v.begin());
  EXPECT_EQ(span.end(), v.end());

  span.at(3) = 9;
  EXPECT_EQ(span.at(3), 9);

  EXPECT_THROW(
    try {
    span.at(-1);
  } catch (std::out_of_range & e) {
    EXPECT_STREQ(e.what(), "Index out of range. -1 < 0");
    throw e;
  }
    ,
    std::out_of_range
  );

  EXPECT_THROW(
    try {
    span.at(7);
  } catch (std::out_of_range & e) {
    EXPECT_STREQ(e.what(), "Index out of range. 7 >= this->size()");
    throw e;
  }
    ,
    std::out_of_range
  );
}

TEST(Span, ConstSpan)
{
  const std::vector<int> v{0, 1, 2, 3, 4, 5, 6};
  const const_span<int> span(v.begin(), v.end());

  EXPECT_EQ(span.size(), 7);
}
