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


#include <gtest/gtest.h>

#include "lidar_feature_localization/irls.hpp"

TEST(IRLS, MedianAbsoluteDeviation)
{
  {
    Eigen::VectorXd v(5);
    v << 7, 9, 3, 0, 1;

    // median(v) = 3
    // | v - median(v) | = [4, 6, 0, 3, 2]
    // median(| v - median(v) |) = 3

    EXPECT_EQ(MedianAbsoluteDeviation(v), 3);
  }

  {
    Eigen::VectorXd v(6);
    v << 8, 3, 4, 0, 5, 1;

    // median(v) = 3.5
    // | v - median(v) | = [4.5, 0.5, 0.5, 3.5, 1.5, 2.5]
    // median(| v - median(v) |) = (1.5 + 2.5) / 2 = 2.0

    EXPECT_EQ(MedianAbsoluteDeviation(v), 2.);
  }
}

TEST(HuberWeights, HuberWeights)
{
  using Vector7d = Eigen::Matrix<double, 1, 7>;
  const Vector7d r(0., 1., -1., 2., -2., 4., -4.);
  const Vector7d weights = HuberWeights(r, 2.);
  const Vector7d expected(1., 1., 1., 1., 1., 0.5, 0.5);

  EXPECT_EQ(weights.size(), r.size());
  EXPECT_EQ((weights - expected).norm(), 0.);
}
