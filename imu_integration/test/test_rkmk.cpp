// Copyright 2022 Ishita Takeshi
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
//    * Neither the name of the Ishita Takeshi nor the names of its
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

#include "imu_integration/rkmk.hpp"

TEST(Psi, Psi)
{
  auto expected = [](const Eigen::Vector3d & u) -> Eigen::Matrix3d {
      const double k = u.norm();
      const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
      const Eigen::Matrix3d U = rotationlib::Hat(u);
      return 0.5 * (I + U + (1. - k / tan(k)) / (k * k) * U * U);
    };

  {
    const Eigen::Vector3d u(0., 0., 0.);
    EXPECT_TRUE((Psi(u) - 0.5 * Eigen::Matrix3d::Identity()).norm() < 1e-6);
  }

  {
    const Eigen::Vector3d u(3. * M_PI / 4., 0., 0.);
    EXPECT_TRUE((Psi(u) - expected(u)).norm() < 1e-6);
  }

  {
    const Eigen::Vector3d u(0., M_PI / 2., 0.);
    EXPECT_TRUE((Psi(u) - expected(u)).norm() < 1e-6);
  }
}
