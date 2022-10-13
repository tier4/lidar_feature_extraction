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

#include <vector>

#include "lidar_feature_extraction/color_points.hpp"

inline void ThrowIfInvalidLabelDetected(const PointLabel & label)
{
  const uint8_t x = static_cast<uint8_t>(label);
  throw std::invalid_argument(fmt::format("Invalid label {}", x));
}

std::vector<uint8_t> LabelToColor(const PointLabel & label)
{
  if (label == PointLabel::Default) {
    return std::vector<uint8_t>{255, 255, 255};
  }
  if (label == PointLabel::Edge) {
    return std::vector<uint8_t>{255, 0, 0};
  }
  if (label == PointLabel::EdgeNeighbor) {
    return std::vector<uint8_t>{255, 63, 0};
  }
  if (label == PointLabel::Surface) {
    return std::vector<uint8_t>{255, 0, 0};
  }
  if (label == PointLabel::SurfaceNeighbor) {
    return std::vector<uint8_t>{255, 63, 0};
  }
  if (label == PointLabel::OutOfRange) {
    return std::vector<uint8_t>{127, 127, 127};
  }
  if (label == PointLabel::Occluded) {
    return std::vector<uint8_t>{255, 0, 255};
  }
  if (label == PointLabel::ParallelBeam) {
    return std::vector<uint8_t>{0, 255, 0};
  }

  ThrowIfInvalidLabelDetected(label);
  return std::vector<uint8_t>{};  // should not reach here
}

std::vector<uint8_t> ValueToColor(const double value, const double min, const double max)
{
  const double v = std::clamp(value, min, max);
  const uint8_t c = static_cast<uint8_t>(255. * v / (max - min));
  return std::vector<uint8_t>{c, c, c};
}
