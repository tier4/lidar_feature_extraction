// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _MATH_LIDAR_ODOMETRY_H_
#define _MATH_LIDAR_ODOMETRY_H_

inline double XYNorm(const double x, const double y)
{
  return std::sqrt(x * x + y * y);
}

double CalcRadian(const double x1, const double y1, const double x2, const double y2)
{
  const double dot = x1 * x2 + y1 * y2;
  const double norm1 = XYNorm(x1, y1);
  const double norm2 = XYNorm(x2, y2);
  const double cos_angle = dot / (norm1 * norm2);
  return std::acos(cos_angle);
}

template<typename T1, typename T2>
double InnerProduct(T1 first1, T1 last1, T2 first2)
{
  double sum = 0.;
  while (first1 != last1) {
    sum += (*first1) * (*first2);
    first1++;
    first2++;
  }
  return sum;
}

#endif  /* _MATH_LIDAR_ODOMETRY_H_ */
