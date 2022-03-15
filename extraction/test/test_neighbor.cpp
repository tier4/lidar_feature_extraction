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


TEST(IsNeighbor, IsNeighbor)
{
  {
    const pcl::PointXYZ p0(1., 1., 0.);
    const pcl::PointXYZ p1(1., 1., 0.);
    EXPECT_TRUE(NeighborCheck(p0, p1, 0.));
  }

  {
    const pcl::PointXYZ p0(0., 1., 0.);
    const pcl::PointXYZ p1(1., 0., 0.);
    EXPECT_TRUE(NeighborCheck(p0, p1, M_PI / 2.));
    EXPECT_FALSE(NeighborCheck(p0, p1, M_PI / 2. - 1e-3));
  }
}

TEST(NeighborCheck, IsNeighbor)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  cloud->push_back(pcl::PointXYZ(1., 1., 0.));
  cloud->push_back(pcl::PointXYZ(0., 1., 0.));
  cloud->push_back(pcl::PointXYZ(1., 0., 0.));
  cloud->push_back(pcl::PointXYZ(1., 0., 0.));

  {
    NeighborCheck is_neighbor(cloud, 0.);
    EXPECT_TRUE(is_neighbor(2, 3));
  }

  {
    NeighborCheck is_neighbor(cloud, M_PI / 4.);
    EXPECT_TRUE(is_neighbor(0, 1));
    EXPECT_FALSE(is_neighbor(1, 2));
  }
}

TEST(NeighborCheck, Size)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

  for (unsigned int i = 0; i < 10; i++) {
    cloud->push_back(pcl::PointXYZ(1., 1., 0.));
  }

  NeighborCheck is_neighbor(cloud, 0.);
  EXPECT_EQ(is_neighbor.Size(), 10);
}
