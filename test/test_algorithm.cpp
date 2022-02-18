// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include <vector>

#include "algorithm.hpp"


TEST(Algorithm, Argsort)
{
  {
    const std::vector<double> curvature{0.3, 0.2, 1.0, 0.2, 0.0, 0.1};
    const std::vector<int> indices = Argsort(curvature);
    EXPECT_THAT(indices, testing::ElementsAre(4, 5, 1, 3, 0, 2));
  }

  {
    const std::vector<double> curvature{0.0, 0.0, 0.0, 0.0, 0.0};
    const std::vector<int> indices = Argsort(curvature);
    EXPECT_THAT(indices, testing::ElementsAre(0, 1, 2, 3, 4));
  }
}
