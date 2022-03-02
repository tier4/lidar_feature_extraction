// Copyright 2022 Tixiao Shan, Takeshi Ishita (2020)
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
//    * Neither the name of the Tixiao Shan, Takeshi Ishita (2020) nor the names of its
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

#include "lidar_feature_mapping/map.hpp"


TEST(Map, TransformAdd)
{
  Map<pcl::PointXYZ> map;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud0(new pcl::PointCloud<pcl::PointXYZ>());
  cloud0->push_back(pcl::PointXYZ(0., 1., 0));
  cloud0->push_back(pcl::PointXYZ(0., 1., 0));
  const Eigen::Affine3d transform0 = Eigen::Affine3d::Identity();

  map.TransformAdd(transform0, cloud0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
  cloud1->push_back(pcl::PointXYZ(0., 1., 0));
  const Eigen::Affine3d transform1(
    Eigen::Translation3d(3., 0., 0.) * Eigen::Quaterniond::Identity());

  map.TransformAdd(transform1, cloud1);

  map.Save(".__testfile.pcd");

  pcl::PointCloud<pcl::PointXYZ> loaded;
  const int retval = pcl::io::loadPCDFile<pcl::PointXYZ> (".__testfile.pcd", loaded);
  ASSERT_EQ(retval, 0);

  auto to_vector = [](const pcl::PointXYZ & p) { return std::vector<double>{p.x, p.y, p.z}; };

  EXPECT_EQ(loaded.size(), static_cast<std::uint32_t>(3));
  EXPECT_THAT(to_vector(loaded.at(0)), testing::ElementsAre(0., 1., 0.));
  EXPECT_THAT(to_vector(loaded.at(1)), testing::ElementsAre(0., 1., 0.));
  EXPECT_THAT(to_vector(loaded.at(2)), testing::ElementsAre(3., 1., 0.));
}


TEST(Map, IsEmpty)
{
  Map<pcl::PointXYZ> map;

  EXPECT_TRUE(map.IsEmpty());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->push_back(pcl::PointXYZ(0., 0., 0));

  map.TransformAdd(Eigen::Affine3d::Identity(), cloud);

  EXPECT_FALSE(map.IsEmpty());
}
