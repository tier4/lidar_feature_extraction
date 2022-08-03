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

#ifndef LIDAR_FEATURE_LIBRARY__EIGEN_HPP_
#define LIDAR_FEATURE_LIBRARY__EIGEN_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

#include <range/v3/all.hpp>


template<int N>
Eigen::MatrixXd VectorsToEigen(const std::vector<Eigen::Matrix<double, N, 1>> & vectors)
{
  Eigen::MatrixXd X(vectors.size(), N);
  for (unsigned int i = 0; i < vectors.size(); i++) {
    X.row(i) = vectors[i];
  }
  return X;
}

template<int N, int M>
Eigen::MatrixXd HorizontalStack(const std::vector<Eigen::Matrix<double, N, M>> & matrices)
{
  Eigen::MatrixXd X(N * matrices.size(), M);
  for (unsigned int i = 0; i < matrices.size(); i++) {
    X.block<N, M>(i * N, 0) = matrices[i];
  }
  return X;
}

Eigen::Isometry3d MakeIsometry3d(const Eigen::Quaterniond & q, const Eigen::Vector3d & t);

Eigen::VectorXd TransformXYZ(const Eigen::Isometry3d & transform, const Eigen::VectorXd & p0);

Eigen::VectorXd VectorToEigen(const std::vector<double> & values);

std::string EigenToString(const Eigen::MatrixXd & matrix);

Eigen::MatrixXd GetRows(
  const Eigen::MatrixXd & matrix,
  const std::vector<std::uint64_t> & indices);

#endif  // LIDAR_FEATURE_LIBRARY__EIGEN_HPP_
