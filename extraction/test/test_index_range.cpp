// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_feature_extraction/index_range.hpp"


TEST(IndexRange, IndexRange)
{
  {
    const IndexRange index_range(0, 12, 3);

    EXPECT_EQ(index_range.NBlocks(), 3);

    EXPECT_EQ(index_range.Begin(0), 0);
    EXPECT_EQ(index_range.End(0), 4);

    EXPECT_EQ(index_range.Begin(1), 4);
    EXPECT_EQ(index_range.End(1), 8);

    EXPECT_EQ(index_range.Begin(2), 8);
    EXPECT_EQ(index_range.End(2), 12);
  }

  {
    const IndexRange index_range(0, 14, 4);

    EXPECT_EQ(index_range.NBlocks(), 4);

    EXPECT_EQ(index_range.Begin(0), 0);
    EXPECT_EQ(index_range.End(0), 3);

    EXPECT_EQ(index_range.Begin(1), 3);
    EXPECT_EQ(index_range.End(1), 7);

    EXPECT_EQ(index_range.Begin(2), 7);
    EXPECT_EQ(index_range.End(2), 10);

    EXPECT_EQ(index_range.Begin(3), 10);
    EXPECT_EQ(index_range.End(3), 14);
  }

  {
    const IndexRange index_range(0, 3, 3);

    EXPECT_EQ(index_range.NBlocks(), 3);

    EXPECT_EQ(index_range.Begin(0), 0);
    EXPECT_EQ(index_range.End(0), 1);

    EXPECT_EQ(index_range.Begin(1), 1);
    EXPECT_EQ(index_range.End(1), 2);

    EXPECT_EQ(index_range.Begin(2), 2);
    EXPECT_EQ(index_range.End(2), 3);
  }

  {
    EXPECT_THROW(
      try {
        IndexRange index_range(1, 3, 3);
      } catch (std::out_of_range & e) {
        EXPECT_STREQ(
          "end_index - start_index (which is 2) cannot be smaller than n_blocks (which is 3)",
          e.what());
        throw e;
      },
      std::invalid_argument);
  }

  {
    const IndexRange index_range(0, 12, 3);

    EXPECT_THROW(
      try {
        index_range.Begin(-1);
      } catch (std::out_of_range & e) {
        EXPECT_STREQ("j (which is -1) < 0 (which is 0)", e.what());
        throw e;
      },
      std::out_of_range);

    EXPECT_THROW(
      try {
        index_range.End(-1);
      } catch (std::out_of_range & e) {
        EXPECT_STREQ("j (which is -1) < 0 (which is 0)", e.what());
        throw e;
      },
      std::out_of_range);

    EXPECT_THROW(
      try {
        index_range.Begin(3);
      } catch (std::out_of_range & e) {
        EXPECT_STREQ("j (which is 3) >= n_blocks (which is 3)", e.what());
        throw e;
      },
      std::out_of_range);

    EXPECT_THROW(
      try {
        index_range.End(3);
      } catch (std::out_of_range & e) {
        EXPECT_STREQ("j (which is 3) >= n_blocks (which is 3)", e.what());
        throw e;
      },
      std::out_of_range);
  }
}

TEST(IndexRange, PaddedIndexRange) {
  {
    const PaddedIndexRange index_range(0, 17, 3, 1);

    EXPECT_EQ(index_range.Begin(0), 0);
    EXPECT_EQ(index_range.End(0), 7);

    EXPECT_EQ(index_range.Begin(1), 5);
    EXPECT_EQ(index_range.End(1), 12);

    EXPECT_EQ(index_range.Begin(2), 10);
    EXPECT_EQ(index_range.End(2), 17);
  }

  {
    const PaddedIndexRange index_range(1, 20, 3, 2);

    EXPECT_EQ(index_range.Begin(0), 1);
    EXPECT_EQ(index_range.End(0), 10);

    EXPECT_EQ(index_range.Begin(1), 6);
    EXPECT_EQ(index_range.End(1), 15);

    EXPECT_EQ(index_range.Begin(2), 11);
    EXPECT_EQ(index_range.End(2), 20);
  }
}

