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

#include "lidar_feature_library/point_type.hpp"

#include "lidar_feature_localization/point_to_vector.hpp"
#include "lidar_feature_localization/pointcloud_to_matrix.hpp"


TEST(PointCloudToMatrix, PointXYZCRToVector)
{
  const pcl::PointCloud<PointXYZCR>::Ptr map(new pcl::PointCloud<PointXYZCR>());

  map->push_back(PointXYZCR(0., 1., 0., 2., 0));
  map->push_back(PointXYZCR(1., 0., 1., 4., 1));
  map->push_back(PointXYZCR(2., 0., 3., 4., 2));

  const Eigen::MatrixXd matrix = PointCloudToMatrix<PointXYZCRToVector, PointXYZCR>(map);

  Eigen::MatrixXd expected(3, 4);
  expected <<
    0., 1., 0., 2.,
    1., 0., 1., 4.,
    2., 0., 3., 4.;

  EXPECT_EQ((matrix - expected).norm(), 0.);
}
