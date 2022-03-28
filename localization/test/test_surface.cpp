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

#include <pcl/common/transforms.h>

#include "lidar_feature_library/transform.hpp"

#include "lidar_feature_localization/optimizer.hpp"
#include "lidar_feature_localization/surface.hpp"


TEST(Surface, EstimatePlaneCoefficients)
{
  {
    const Eigen::MatrixXd X =
      (Eigen::MatrixXd(4, 2) <<
        2, 3,
        3, 4,
        4, 5,
        5, 6
      ).finished();

    const Eigen::Vector2d w = EstimatePlaneCoefficients(X);
    const Eigen::Vector2d expected(1, -1);
    EXPECT_THAT((w - expected).norm(), testing::Le(1e-6));
  }

  {
    const Eigen::MatrixXd X =
      (Eigen::MatrixXd(4, 3) <<
        1, 1, -1,
        2, 1, -2,
        2, -1, 0,
        4, -2, -1
      ).finished();
    const Eigen::Vector3d w = EstimatePlaneCoefficients(X);
    const Eigen::Vector3d expected(1, 1, 1);
    const Eigen::Vector4d bias = Eigen::Vector4d::Ones();
    EXPECT_THAT((X * w + bias).norm(), testing::Le(1e-6));
  }
}

TEST(Surface, CheckPointsDistributeAlongPlane)
{
  {
    const Eigen::MatrixXd X =
      (Eigen::MatrixXd(4, 2) <<
        2, 3,
        3, 4,
        4, 5,
        5, 6
      ).finished();
    const Eigen::Vector2d w(1, -1);
    EXPECT_TRUE(CheckPointsDistributeAlongPlane(X, w));
  }

  {
    const Eigen::MatrixXd X =
      (Eigen::MatrixXd(4, 2) <<
        2, 3,
        3, 4,
        5, 5,
        5, 6
      ).finished();
    const Eigen::Vector2d w(1, -1);
    EXPECT_FALSE(CheckPointsDistributeAlongPlane(X, w));
  }
}
