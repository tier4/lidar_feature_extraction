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

#include "lidar_feature_extraction/curvature.hpp"

TEST(Curvature, MakeWeight)
{
  EXPECT_THAT(MakeWeight(2), testing::ElementsAre(1., 1., -4., 1., 1.));
  EXPECT_THAT(MakeWeight(3), testing::ElementsAre(1., 1., 1., -6., 1., 1., 1.));
}

TEST(Curvature, CalcCurvature)
{
  {
    const std::vector<double> range{1., 1., 2., 0., 1., 1., 0.};
    const int padding = 2;
    const std::vector<double> result = CalcCurvature(range, padding);

    const double e0 = 1 * 1 + 1 * 1 + 2 * (-4) + 0 * 1 + 1 * 1;
    const double e1 = 1 * 1 + 2 * 1 + 0 * (-4) + 1 * 1 + 1 * 1;
    const double e2 = 2 * 1 + 0 * 1 + 1 * (-4) + 1 * 1 + 0 * 1;

    EXPECT_THAT(result.size(), range.size());
    EXPECT_THAT(result, testing::ElementsAre(0., 0., e0 * e0, e1 * e1, e2 * e2, 0., 0.));
  }

  {
    const std::vector<double> range{4., 4., 1., 2., 0., 5., 3., 6.};
    const int padding = 3;
    const std::vector<double> result = CalcCurvature(range, padding);

    const double e0 = 4 * 1 + 4 * 1 + 1 * 1 + 2 * (-6) + 0 * 1 + 5 * 1 + 3 * 1;
    const double e1 = 4 * 1 + 1 * 1 + 2 * 1 + 0 * (-6) + 5 * 1 + 3 * 1 + 6 * 1;

    EXPECT_THAT(result.size(), range.size());
    EXPECT_THAT(result, testing::ElementsAre(0., 0., 0., e0 * e0, e1 * e1, 0., 0., 0.));
  }
}
