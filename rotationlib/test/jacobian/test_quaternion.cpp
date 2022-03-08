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

#include "rotationlib/jacobian/quaternion.hpp"
#include "near_with_precision.hpp"

using testing::Pointwise;

Eigen::Matrix4d Q(const Eigen::Quaterniond & q)
{
  const double w = q.w();
  const double x = q.x();
  const double y = q.y();
  const double z = q.z();

  Eigen::Matrix4d Q;
  Q <<
    w, -x, -y, -z,
    x, w, -z, y,
    y, z, w, -x,
    z, -y, x, w;
  return Q;
}

Eigen::Matrix4d P(const Eigen::Quaterniond & p)
{
  const double w = p.w();
  const double x = p.x();
  const double y = p.y();
  const double z = p.z();

  Eigen::Matrix4d P;
  P <<
    w, -x, -y, -z,
    x, w, z, -y,
    y, -z, w, x,
    z, y, -x, w;
  return P;
}

TEST(Quaternion, DRpDq)
{
  const Eigen::Vector3d p(1.0, -1.0, 2.0);
  const Eigen::Quaterniond q = Eigen::Quaterniond(1.0, -1.0, -1.0, 1.0).normalized();
  const Eigen::Matrix<double, 3, 4> J = rotationlib::DRpDq(q, p);

  const Eigen::Matrix4d inv_operator = Eigen::Vector4d(1., -1., -1., -1.).asDiagonal();
  const Eigen::Quaterniond u = Eigen::Quaterniond(0., p(0), p(1), p(2));
  const Eigen::Matrix4d D = Q(q * u) * inv_operator + P(u * q.inverse());
  EXPECT_EQ((D.bottomRows(3) - J).norm(), 0.);
}
