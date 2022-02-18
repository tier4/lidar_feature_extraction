// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include "math.hpp"

TEST(Utility, XYNorm)
{
  EXPECT_EQ(XYNorm(0., 0.), 0.);
  EXPECT_EQ(XYNorm(-1., 0.), 1.);
  EXPECT_EQ(XYNorm(3., 4.), 5.);
}
