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

#include <Eigen/Core>

#include <cmath>
#include <unordered_set>

#include "lidar_feature_library/random.hpp"


TEST(RandomizedUniqueIndices, SmokeTest)
{
  std::srand(3939);

  const size_t size = 40;
  const std::vector<size_t> indices = RandomizedUniqueIndices(size);

  ASSERT_EQ(indices.size(), size);

  const size_t min = *std::min_element(indices.begin(), indices.end());
  const size_t max = *std::max_element(indices.begin(), indices.end());
  ASSERT_EQ(min, 0U);
  ASSERT_EQ(max, size-1);

  // checks if indices are randomized
  ASSERT_NE(indices.at(0), 0U);
  ASSERT_NE(indices.at(size-1), size-1);

  std::unordered_set<size_t> unique;
  for (const size_t i : indices) {
    unique.insert(i);
  }

  ASSERT_EQ(unique.size(), size);
}

TEST(RandomizedUniqueIndices, Empty)
{
  ASSERT_EQ(RandomizedUniqueIndices(0).size(), 0U);
}

constexpr double tolerance = 1e-2;
constexpr int N = 100000;

double StandardDeviation(const Eigen::VectorXd & v)
{
  const double n = static_cast<double>(v.size());
  return std::sqrt((v.array() - v.mean()).square().sum() / (n - 1.));
}

TEST(Random, NormalDistribution)
{
  auto generate = [&](const double mean, const double stddev) {
      NormalDistribution<double> normal(mean, stddev);

      Eigen::VectorXd v(N);
      for (size_t i = 0; i < N; i++) {
        v(i) = normal();
      }
      return v;
    };

  {
    const double mean = 0.;
    const double stddev = 1.0;

    const Eigen::VectorXd v = generate(mean, stddev);
    EXPECT_NEAR(v.mean(), mean, tolerance);
    EXPECT_NEAR(StandardDeviation(v), stddev, tolerance);
  }

  {
    const double mean = 2.;
    const double stddev = 0.3;

    const Eigen::VectorXd v = generate(mean, stddev);
    EXPECT_NEAR(v.mean(), mean, tolerance);
    EXPECT_NEAR(StandardDeviation(v), stddev, tolerance);
  }
}
