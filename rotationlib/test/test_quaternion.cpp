// Copyright 2022 Takeshi Ishita
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
//    * Neither the name of the Takeshi Ishita nor the names of its
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

#include "rotationlib/quaternion.hpp"


using rotationlib::FromWXYZ;
using rotationlib::ToWXYZ;
using rotationlib::LeftMultiplicationMatrix;
using rotationlib::RightMultiplicationMatrix;
using rotationlib::RPYToQuaternionXYZ;


TEST(Quaternion, LeftMultiplicationMatrix)
{
  const Eigen::Quaterniond q1 = FromWXYZ(Eigen::Vector4d::Random());
  const Eigen::Quaterniond q2 = FromWXYZ(Eigen::Vector4d::Random());
  const Eigen::Vector4d q3 = ToWXYZ(q1 * q2);
  const Eigen::Vector4d q4 = LeftMultiplicationMatrix(q1) * ToWXYZ(q2);
  EXPECT_THAT((q3 - q4).norm(), testing::Le(1e-4));
}

TEST(Quaternion, RightMultiplicationMatrix)
{
  const Eigen::Quaterniond q1 = FromWXYZ(Eigen::Vector4d::Random());
  const Eigen::Quaterniond q2 = FromWXYZ(Eigen::Vector4d::Random());
  const Eigen::Vector4d q3 = ToWXYZ(q1 * q2);
  const Eigen::Vector4d q4 = RightMultiplicationMatrix(q2) * ToWXYZ(q1);
  EXPECT_THAT((q3 - q4).norm(), testing::Le(1e-4));
}

TEST(Eigen, RPYToQuaternionXYZ)
{
  const double roll = 0.1;
  const double pitch = 0.2;
  const double yaw = 0.3;

  // obtained from python
  // >>> from scipy.spatial.transform import Rotation
  // >>> [x, y, z, w] = Rotation.from_euler('xyz', [0.1, 0.2, 0.3]).as_quat()

  const double w = 0.9833474432563558;
  const double x = 0.034270798550482096;
  const double y = 0.10602051106179562;
  const double z = 0.1435721750273919;

  const Eigen::Quaterniond q = RPYToQuaternionXYZ(roll, pitch, yaw);

  const double tolerance = 1e-8;
  EXPECT_NEAR(q.w(), w, tolerance);
  EXPECT_NEAR(q.x(), x, tolerance);
  EXPECT_NEAR(q.y(), y, tolerance);
  EXPECT_NEAR(q.z(), z, tolerance);
}
