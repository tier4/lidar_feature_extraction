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

#ifndef POSEVEC_HPP_
#define POSEVEC_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "lidar_feature_localization/matrix_type.hpp"


Eigen::Isometry3d MakePose(const Vector7d & posevec)
{
  const Eigen::Vector4d wxyz = posevec.head(4);
  const Eigen::Quaterniond q(wxyz(0), wxyz(1), wxyz(2), wxyz(3));

  Eigen::Isometry3d pose;
  pose.linear() = q.toRotationMatrix();
  pose.translation() = posevec.tail(3);
  return pose;
}

Vector7d MakePosevec(const Eigen::Isometry3d & pose)
{
  const Eigen::Quaterniond q = Eigen::Quaterniond(pose.rotation()).normalized();
  const Eigen::Vector3d t = pose.translation();
  Vector7d posevec;
  posevec << q.w(), q.x(), q.y(), q.z(), t.x(), t.y(), t.z();
  return posevec;
}

Eigen::Quaterniond Exp(const Eigen::Vector3d & r)
{
  const double norm = r.norm();

  if (norm < 1e-7) {
    return Eigen::Quaterniond::Identity();
  }

  const Eigen::Vector3d xyz = r * std::sin(norm) / norm;
  const double w = std::cos(norm);

  return Eigen::Quaterniond(w, xyz(0), xyz(1), xyz(2));
}

Eigen::Quaterniond UpdateQuaternionVec(
  const Eigen::Vector4d & current,
  const Eigen::Vector4d & dwxyz)
{
  const Eigen::Quaterniond q(current(0), current(1), current(2), current(3));

  const Eigen::Quaterniond dq = Exp(Eigen::Vector3d(dwxyz(1), dwxyz(2), dwxyz(3)));

  return dq * q;
}

Vector7d UpdatePoseVec(const Vector7d & x0, const Vector7d & dx)
{
  const Eigen::Quaterniond q = UpdateQuaternionVec(x0.head(4), dx.head(4));

  Vector7d x1;
  x1.head(4) = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
  x1.tail(3) = x0.tail(3) + dx.tail(3);
  return x1;
}

#endif  // POSEVEC_HPP_
