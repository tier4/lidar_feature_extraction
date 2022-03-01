// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

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
    } catch(const std::invalid_argument & e) {
      EXPECT_STREQ("All input values are zero. Angle cannot be calculated", e.what());
      throw e;
    },
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
