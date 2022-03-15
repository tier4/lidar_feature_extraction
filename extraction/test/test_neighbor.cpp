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

#include "lidar_feature_extraction/neighbor.hpp"
#include "lidar_feature_extraction/iterator.hpp"


TEST(IsNeighbor, IsNeighbor)
{
  {
    const pcl::PointXYZ p0(1., 1., 0.);
    const pcl::PointXYZ p1(1., 1., 0.);
    EXPECT_TRUE(IsNeighbor(p0, p1, 0.));
  }

  {
    const pcl::PointXYZ p0(0., 1., 0.);
    const pcl::PointXYZ p1(1., 0., 0.);
    EXPECT_TRUE(IsNeighbor(p0, p1, M_PI / 2.));
    EXPECT_FALSE(IsNeighbor(p0, p1, M_PI / 2. - 1e-3));
  }
}

TEST(NeighborCheck, NeighborCheck)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  cloud->push_back(pcl::PointXYZ(1., 1., 0.));
  cloud->push_back(pcl::PointXYZ(0., 1., 0.));
  cloud->push_back(pcl::PointXYZ(1., 0., 0.));
  cloud->push_back(pcl::PointXYZ(1., 0., 0.));
  MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));

  {
    const NeighborCheck is_neighbor(ref_points, 0.);
    EXPECT_EQ(is_neighbor.Size(), 4);
  }

  {
    const NeighborCheck is_neighbor(ref_points, 0.);
    EXPECT_TRUE(is_neighbor(2, 3));
  }

  {
    const NeighborCheck is_neighbor(ref_points, M_PI / 4.);
    EXPECT_TRUE(is_neighbor(0, 1));
    EXPECT_FALSE(is_neighbor(1, 2));
  }

  {
    const NeighborCheck is_neighbor(ref_points, M_PI / 4.);
    const NeighborCheck sliced = is_neighbor.Slice(1, 3);
    EXPECT_EQ(sliced.Size(), 2);
    EXPECT_FALSE(sliced(0, 1));
  }
}

TEST(NeighborCheck, ThrowIfInsufficientPoints)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->push_back(pcl::PointXYZ(1., 1., 0.));

  MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));

  EXPECT_THROW(
    try {
      const NeighborCheck is_neighbor(ref_points, 0.);
      EXPECT_EQ(is_neighbor.Size(), 4);
    } catch(std::invalid_argument & e) {
      EXPECT_STREQ(e.what(), "The input point size (which is 1) cannot be smaller than 2");
      throw e;
    },
    std::invalid_argument
  );
}
