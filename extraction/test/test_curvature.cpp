// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#include <gmock/gmock.h>

#include "lidar_feature_extraction/curvature.hpp"

TEST(Curvature, MakeWeight)
{
  EXPECT_THAT(MakeWeight(2), testing::ElementsAre(1., 1., -4., 1., 1.));
  EXPECT_THAT(MakeWeight(3), testing::ElementsAre(1., 1., 1., -6., 1., 1., 1.));
}

TEST(Curvature, CalcCurvature)
{
  const std::vector<double> range{1., 1., 2., 0., 1., 1., 0.};
  const int padding = 2;
  const std::vector<double> result = CalcCurvature(range, padding);

  const double e0 = 1 * 1 + 1 * 1 + 2 * (-4) + 0 * 1 + 1 * 1;
  const double e1 = 1 * 1 + 2 * 1 + 0 * (-4) + 1 * 1 + 1 * 1;
  const double e2 = 2 * 1 + 0 * 1 + 1 * (-4) + 1 * 1 + 0 * 1;

  EXPECT_THAT(result, testing::ElementsAre(e0 * e0, e1 * e1, e2 * e2));
}

TEST(Curvature, Curvature)
{
  const double edge_threshold = 0.3;
  const double surface_threshold = 0.2;

  const std::vector<double> curvature_values{0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  const Curvature curvature(curvature_values, edge_threshold, surface_threshold);

  ASSERT_THAT(curvature.Size(), curvature_values.size());

  std::vector<bool> is_edge(curvature.Size());
  for (int i = 0; i < curvature.Size(); i++) {
    is_edge[i] = curvature.IsEdge(i);
  }

  std::vector<bool> is_surface(curvature.Size());
  for (int i = 0; i < curvature.Size(); i++) {
    is_surface[i] = curvature.IsSurface(i);
  }

  EXPECT_THAT(is_edge, testing::ElementsAre(false, false, false, true, true, true));

  EXPECT_THAT(is_surface, testing::ElementsAre(true, true, true, false, false, false));
}

