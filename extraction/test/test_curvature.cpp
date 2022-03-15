// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

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
