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

#include "lidar_feature_localization/posevec.hpp"

TEST(Posevec, MakePosevec)
{
  Eigen::Matrix3d R;
  R <<
    0.,  0., -1.,
    1.,  0.,  0.,
    0., -1.,  0.;

  Eigen::Vector3d t(9., 2., 4.);

  Eigen::Isometry3d pose;
  pose.linear() = R;
  pose.translation() = t;

  const Vector7d posevec = MakePosevec(pose);
  Vector7d expected;
  expected << -0.5,  0.5,  0.5, -0.5, 9., 2., 4.;
  EXPECT_THAT((posevec - expected).norm(), testing::Le(1e-3));
}

TEST(Posevec, MakePose)
{

  Vector7d posevec;
  posevec << -0.5,  0.5,  0.5, -0.5, 9, 2, 4;
  const Eigen::Isometry3d pose = MakePose(posevec);
  Eigen::Matrix3d R;
  R <<
    0.,  0., -1.,
    1.,  0.,  0.,
    0., -1.,  0.;

  Eigen::Vector3d t(9., 2., 4.);

  EXPECT_THAT((pose.rotation() - R).norm(), testing::Le(1e-3));
  EXPECT_EQ((pose.translation() - t).norm(), 0.);
}
