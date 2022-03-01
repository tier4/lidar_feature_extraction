// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include "lidar_feature_extraction/range_message.hpp"


TEST(RangeMessage, RangeMessageLargerThanOrEqualTo)
{
  EXPECT_EQ(
    RangeMessageLargerThanOrEqualTo("i", "max", 39, 30),
    "i (which is 39) >= max (which is 30)");
}

TEST(RangeMessage, RangeMessageSmallerThanOrEqualTo)
{
  EXPECT_EQ(
    RangeMessageSmallerThanOrEqualTo("i", "min", 39, 40),
    "i (which is 39) <= min (which is 40)");
}

TEST(RangeMessage, RangeMessageLargerThan)
{
  EXPECT_EQ(
    RangeMessageLargerThan("i", "max", 39, 30),
    "i (which is 39) > max (which is 30)");
}

TEST(RangeMessage, RangeMessageSmallerThan)
{
  EXPECT_EQ(
    RangeMessageSmallerThan("i", "min", 39, 40),
    "i (which is 39) < min (which is 40)");
}
