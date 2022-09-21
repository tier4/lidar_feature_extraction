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

#include "lidar_feature_library/stats.hpp"

#include <vector>

// private method, because the argument will be modified
double Median_(std::vector<double> & v)
{
  if (v.size() == 0) {
    throw std::invalid_argument("Empty array is passed to the median function");
  }

  if (v.size() % 2 == 1) {
    const int n = (v.size() - 1) / 2;
    std::nth_element(v.begin(), v.begin() + n, v.end());
    return v[n];
  }

  const int n = v.size() / 2;

  std::nth_element(v.begin(), v.begin() + n - 0, v.end());
  const double e0 = v[n - 0];

  std::nth_element(v.begin(), v.begin() + n - 1, v.end());
  const double e1 = v[n - 1];

  return (e0 + e1) / 2.;
}

double Median(const Eigen::VectorXd & m)
{
  std::vector<double> v(m.begin(), m.end());
  return Median_(v);
}

double Median(const Eigen::ArrayXd & m)
{
  std::vector<double> v(m.begin(), m.end());
  return Median_(v);
}
