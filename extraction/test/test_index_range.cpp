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
    }
      ,
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
    }
      ,
      std::out_of_range);

    EXPECT_THROW(
      try {
      index_range.End(-1);
    } catch (std::out_of_range & e) {
      EXPECT_STREQ("j (which is -1) < 0 (which is 0)", e.what());
      throw e;
    }
      ,
      std::out_of_range);

    EXPECT_THROW(
      try {
      index_range.Begin(3);
    } catch (std::out_of_range & e) {
      EXPECT_STREQ("j (which is 3) >= n_blocks (which is 3)", e.what());
      throw e;
    }
      ,
      std::out_of_range);

    EXPECT_THROW(
      try {
      index_range.End(3);
    } catch (std::out_of_range & e) {
      EXPECT_STREQ("j (which is 3) >= n_blocks (which is 3)", e.what());
      throw e;
    }
      ,
      std::out_of_range);
  }
}

TEST(IndexRange, PaddedIndexRange) {
  {
    const PaddedIndexRange index_range(17, 3, 1);

    EXPECT_EQ(index_range.Begin(0), 1);
    EXPECT_EQ(index_range.End(0), 6);

    EXPECT_EQ(index_range.Begin(1), 6);
    EXPECT_EQ(index_range.End(1), 11);

    EXPECT_EQ(index_range.Begin(2), 11);
    EXPECT_EQ(index_range.End(2), 16);
  }

  {
    const PaddedIndexRange index_range(20, 4, 2);

    EXPECT_EQ(index_range.Begin(0), 2);
    EXPECT_EQ(index_range.End(0), 6);

    EXPECT_EQ(index_range.Begin(1), 6);
    EXPECT_EQ(index_range.End(1), 10);

    EXPECT_EQ(index_range.Begin(2), 10);
    EXPECT_EQ(index_range.End(2), 14);

    EXPECT_EQ(index_range.Begin(3), 14);
    EXPECT_EQ(index_range.End(3), 18);
  }
}
