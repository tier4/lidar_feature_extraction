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

#include "lidar_feature_extraction/math.hpp"


TEST(Math, XYNorm)
{
  EXPECT_EQ(XYNorm(0., 0.), 0.);
  EXPECT_EQ(XYNorm(-1., 0.), 1.);
  EXPECT_EQ(XYNorm(3., 4.), 5.);
}

TEST(Math, CalcRadian)
{
  const double threshold = 1e-7;

  EXPECT_NEAR(CalcRadian(1., 1., 1., 1.), 0., threshold);
  EXPECT_NEAR(CalcRadian(-1., 1., -1., 1.), 0., threshold);
  EXPECT_NEAR(CalcRadian(1., 0., 0., 1.), M_PI / 2., threshold);
  EXPECT_NEAR(CalcRadian(0., 1., 1., 0.), M_PI / 2., threshold);
  EXPECT_NEAR(CalcRadian(1., -1., 1., 1.), M_PI / 2., threshold);
  EXPECT_NEAR(CalcRadian(-1., -1., 1., 1.), M_PI, threshold);
  EXPECT_NEAR(CalcRadian(1., 1., -1., -1.), M_PI, threshold);
  EXPECT_NEAR(CalcRadian(-1., 1., 0., 1.), M_PI / 4., threshold);
  EXPECT_NEAR(CalcRadian(0., 1., -1., 1.), M_PI / 4., threshold);

  EXPECT_THROW(
    try {
    CalcRadian(0., 0., 0., 0.);
  } catch (const std::invalid_argument & e) {
    EXPECT_STREQ("All input values are zero. Angle cannot be calculated", e.what());
    throw e;
  }
    ,
    std::invalid_argument);
}

TEST(Math, InnerProduct)
{
  std::vector<int> a{1, 0, 2, 4};
  std::vector<int> b{3, 1, 0, 2};

  EXPECT_EQ(InnerProduct(a.begin(), a.end(), b.begin()), 11);

  std::vector<int> c{1, 4};
  std::vector<int> d{3, 1};

  EXPECT_EQ(InnerProduct(c.begin(), c.end(), d.begin()), 7);
}
