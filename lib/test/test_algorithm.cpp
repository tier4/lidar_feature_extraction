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

#include "lidar_feature_library/algorithm.hpp"


TEST(Algorithm, GetIndicesByValue)
{
  std::vector<int> array{0, 3, 2, 3, 4};
  EXPECT_THAT(GetIndicesByValue(array, 3), testing::ElementsAre(1, 3));
  EXPECT_THAT(GetIndicesByValue(array, 2), testing::ElementsAre(2));
}

TEST(Algorithm, GetByIndices)
{
  std::vector<int> array{0, 3, 2, 3, 5};
  EXPECT_THAT(GetByIndices(std::vector<size_t>{0, 1}, array), testing::ElementsAre(0, 3));
  EXPECT_THAT(GetByIndices(std::vector<size_t>{2, 4}, array), testing::ElementsAre(2, 5));
}

TEST(Algorithm, SortThreeValues)
{
  {
    Eigen::Vector3d expected(0, 1, 2);
    EXPECT_EQ((SortThreeValues(Eigen::Vector3d(0, 1, 2)) - expected).norm(), 0);
    EXPECT_EQ((SortThreeValues(Eigen::Vector3d(0, 2, 1)) - expected).norm(), 0);
    EXPECT_EQ((SortThreeValues(Eigen::Vector3d(1, 0, 2)) - expected).norm(), 0);
    EXPECT_EQ((SortThreeValues(Eigen::Vector3d(1, 2, 0)) - expected).norm(), 0);
    EXPECT_EQ((SortThreeValues(Eigen::Vector3d(2, 0, 1)) - expected).norm(), 0);
    EXPECT_EQ((SortThreeValues(Eigen::Vector3d(2, 1, 0)) - expected).norm(), 0);
  }

  {
    Eigen::Vector3d expected(0, 1, 1);
    EXPECT_EQ((SortThreeValues(Eigen::Vector3d(1, 1, 0)) - expected).norm(), 0);
  }

  {
    Eigen::Vector3d expected(0, 0, 0);
    EXPECT_EQ((SortThreeValues(Eigen::Vector3d(0, 0, 0)) - expected).norm(), 0);
  }
}
