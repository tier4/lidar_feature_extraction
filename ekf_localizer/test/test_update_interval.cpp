// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <gtest/gtest.h>

#include "ekf_localizer/update_interval.hpp"


constexpr double threshold = 1e-8;


TEST(UpdateInterval, SmokeTest)
{
  UpdateInterval interval(10.);

  const double dt1 = interval.Compute(10000.10);
  EXPECT_NEAR(0.1, dt1, threshold);

  const double dt2 = interval.Compute(10000.20);
  EXPECT_NEAR(0.1, dt2, threshold);

  const double dt3 = interval.Compute(10000.50);
  EXPECT_NEAR(0.3, dt3, threshold);

  const double dt4 = interval.Compute(10000.70);
  EXPECT_NEAR(0.2, dt4, threshold);
}

TEST(UpdateInterval, DetectJumpBackInTime)
{
  UpdateInterval interval(10.);

  interval.Compute(10000.50);

  EXPECT_THROW(
    try {
      interval.Compute(10000.49);
    } catch(std::invalid_argument & e) {
      EXPECT_STREQ("Detected jump back in time", e.what());
      throw e;
    },
    std::invalid_argument
  );
}

TEST(UpdateInterval, MaxInterval) {
  UpdateInterval interval(0.09);

  const double dt1 = interval.Compute(10000.0);
  EXPECT_NEAR(10.0, dt1, threshold);

  const double dt2 = interval.Compute(10011.00);
  EXPECT_NEAR(10.0, dt2, threshold);
}
