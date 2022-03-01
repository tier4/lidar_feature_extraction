// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include "lidar_feature_extraction/iterator.hpp"


TEST(Iterator, Irange)
{
  EXPECT_THAT(irange(4), testing::ElementsAre(0, 1, 2, 3));
}
