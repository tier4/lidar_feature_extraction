// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

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
      },
      std::invalid_argument);
  }
}
