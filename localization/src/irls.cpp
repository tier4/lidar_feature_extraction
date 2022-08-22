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

#include <Eigen/Core>

#include <cmath>

#include "lidar_feature_library/stats.hpp"
#include "lidar_feature_localization/irls.hpp"


double MedianAbsoluteDeviation(const Eigen::VectorXd & v)
{
  const double median = Median(v);
  return Median((v.array() - median).abs().eval());
}

double Scale(const Eigen::VectorXd & v)
{
  // >>> from scipy.stats import norm
  // >>> 1 / norm.ppf(3 / 4)
  // 1.482602218505602

  const double b = 1.482602218505602;
  return b * MedianAbsoluteDeviation(v);
}

Eigen::VectorXd HuberWeights(const Eigen::VectorXd & residuals, const double k)
{
  auto compute = [&k](const double r) {
      const double abs_r = std::fabs(r);
      if (abs_r <= k) {
        return 1.;
      }
      return k / abs_r;
    };

  Eigen::VectorXd weights(residuals.size());

  for (int64_t i = 0; i < residuals.size(); i++) {
    weights(i) = compute(residuals(i));
  }

  return weights;
}
