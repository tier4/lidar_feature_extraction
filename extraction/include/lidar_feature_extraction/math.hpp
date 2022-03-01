// Copyright 2022 Tixiao Shan, Takeshi Ishita (2020)
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
//    * Neither the name of the Tixiao Shan, Takeshi Ishita (2020) nor the names of its
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


#ifndef MATH_HPP_
#define MATH_HPP_

inline double XYNorm(const double x, const double y)
{
  return std::sqrt(x * x + y * y);
}

double CalcRadian(const double x1, const double y1, const double x2, const double y2)
{
  const double dot = x1 * x2 + y1 * y2;
  const double norm1 = XYNorm(x1, y1);
  const double norm2 = XYNorm(x2, y2);

  if (norm1 == 0 && norm2 == 0) {
    throw std::invalid_argument("All input values are zero. Angle cannot be calculated");
  }

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

#endif  // MATH_HPP_
