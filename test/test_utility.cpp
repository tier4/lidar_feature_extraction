#include <gtest/gtest.h>
#include "utility.hpp"

using PointField = sensor_msgs::msg::PointField;

TEST(Utility, RingIsAvailable)
{
  const std::vector<sensor_msgs::msg::PointField> with_ring = {
    PointField().set__name("intensity").set__offset(16).set__datatype(7).set__count(1),
    PointField().set__name("ring").set__offset(20).set__datatype(4).set__count(1)
  };
  EXPECT_TRUE(RingIsAvailable(with_ring));

  const std::vector<sensor_msgs::msg::PointField> without_ring = {
    PointField().set__name("intensity").set__offset(16).set__datatype(7).set__count(1)
  };
  EXPECT_FALSE(RingIsAvailable(without_ring));
}
