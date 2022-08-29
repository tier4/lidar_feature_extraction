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

#include <cmath>
#include <random>

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

TEST(Scale, StandardDeviation)
{
  // generate random numbers, compute the sample variance, and compare to the
  // sample variance

  const double mean = 0.;
  const double stddev = 10.;
  const int n = 100000;

  const Eigen::VectorXd errors = [&] {
      std::random_device rd{};
      std::mt19937 gen{rd()};

      std::normal_distribution<> normal{mean, stddev};

      Eigen::VectorXd errors(n);
      for (int i = 0; i < n; ++i) {
        errors(i) = normal(gen);
      }
      return errors;
    }();

  const double nf = static_cast<double>(n);
  const double sample_stddev = std::sqrt(((nf - 1) / nf) * (stddev * stddev));
  const double scale = Scale(errors);

  EXPECT_LE(std::fabs(sample_stddev - scale), 0.05);
}

TEST(HuberDerivative, NumericalDiff)
{
  const double k = 1.0;

  {
    const double d = (Huber(0.90 + 1e-4, k) - Huber(0.90, k)) / 1e-4;
    ASSERT_LT(std::fabs(d - HuberDerivative(0.90)), 1e-3);
  }
}
