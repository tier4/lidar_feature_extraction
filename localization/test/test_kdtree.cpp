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

#include "lidar_feature_localization/kdtree.hpp"


TEST(KDTree, KDTree)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->push_back(pcl::PointXYZ(2, 0, 1)),
  cloud->push_back(pcl::PointXYZ(2, 0, 0)),
  cloud->push_back(pcl::PointXYZ(0, 0, 4)),
  cloud->push_back(pcl::PointXYZ(0, 2, 4));

  const KDTree<pcl::PointXYZ> kdtree(cloud);

  {
    const auto [indices, squared_distances] = kdtree.RadiusSearch(pcl::PointXYZ(2, 0, 1), 1.0);
    EXPECT_THAT(indices, testing::ElementsAre(0));
    EXPECT_THAT(squared_distances, testing::ElementsAre(0.));
  }

  {
    const auto [indices, squared_distances] = kdtree.RadiusSearch(pcl::PointXYZ(2, 0, 1), 1.2);
    EXPECT_THAT(indices, testing::ElementsAre(0, 1));
    EXPECT_THAT(squared_distances, testing::ElementsAre(0., 1.));
  }

  {
    const auto [indices, squared_distances] = kdtree.NearestKSearch(pcl::PointXYZ(0, 0, 0), 2);
    EXPECT_THAT(indices, testing::ElementsAre(1, 0));
    EXPECT_THAT(squared_distances, testing::ElementsAre(4., 5.));
  }

  {
    const auto [indices, squared_distances] = kdtree.NearestKSearch(pcl::PointXYZ(0, 0, 0), 4);
    EXPECT_THAT(indices, testing::ElementsAre(1, 0, 2, 3));
    EXPECT_THAT(squared_distances, testing::ElementsAre(4., 5., 16., 20.));
  }
}

TEST(KDTree, KDTreeEigen)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->push_back(pcl::PointXYZ(2, 0, 1)),
  cloud->push_back(pcl::PointXYZ(2, 0, 0)),
  cloud->push_back(pcl::PointXYZ(0, 0, 4)),
  cloud->push_back(pcl::PointXYZ(0, 2, 4));

  const KDTreeEigen kdtree(cloud);

  {
    const auto [X, squared_distances] = kdtree.RadiusSearch(Eigen::Vector3d(2, 0, 1), 1.0);
    const Eigen::MatrixXd expected = (Eigen::MatrixXd(1, 3) << 2, 0, 1).finished();
    ASSERT_THAT(X.rows(), 1);
    ASSERT_THAT(X.cols(), 3);
    EXPECT_THAT(squared_distances, testing::ElementsAre(0.));
  }

  {
    const auto [X, squared_distances] = kdtree.RadiusSearch(Eigen::Vector3d(2, 0, 1), 1.2);
    const Eigen::MatrixXd expected =
      (Eigen::MatrixXd(2, 3) <<
      2, 0, 1,
      2, 0, 0
      ).finished();
    ASSERT_THAT(X.rows(), 2);
    ASSERT_THAT(X.cols(), 3);
    EXPECT_THAT((X - expected).norm(), 0.);
    EXPECT_THAT(squared_distances, testing::ElementsAre(0., 1.));
  }

  {
    const auto [X, squared_distances] = kdtree.NearestKSearch(Eigen::Vector3d(0, 0, 0), 2);
    const Eigen::MatrixXd expected =
      (Eigen::MatrixXd(2, 3) <<
      2, 0, 0,
      2, 0, 1
      ).finished();
    ASSERT_THAT(X.rows(), 2);
    ASSERT_THAT(X.cols(), 3);
    EXPECT_EQ((X - expected).norm(), 0.);
    EXPECT_THAT(squared_distances, testing::ElementsAre(4., 5.));
  }

  {
    const auto [X, squared_distances] = kdtree.NearestKSearch(Eigen::Vector3d(0, 0, 0), 4);

    const Eigen::MatrixXd expected =
      (Eigen::MatrixXd(4, 3) <<
      2, 0, 0,
      2, 0, 1,
      0, 0, 4,
      0, 2, 4
      ).finished();
    ASSERT_THAT(X.rows(), 4);
    ASSERT_THAT(X.cols(), 3);
    EXPECT_EQ((X - expected).norm(), 0.);
    EXPECT_THAT(squared_distances, testing::ElementsAre(4., 5., 16., 20.));
  }
}
