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

#include <gmock/gmock.h>

#include "lidar_feature_extraction/out_of_range.hpp"


TEST(Label, LabelOutOfRange)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->push_back(pcl::PointXYZ(1.9, 0.0, 0.0));
  cloud->push_back(pcl::PointXYZ(2.0, 0.0, 0.0));
  cloud->push_back(pcl::PointXYZ(0.0, 5.0, 0.0));
  cloud->push_back(pcl::PointXYZ(0.0, 8.0, 0.0));
  cloud->push_back(pcl::PointXYZ(0.0, 8.1, 0.0));

  const MappedPoints<pcl::PointXYZ> ref_points(cloud, irange(cloud->size()));
  const Range<pcl::PointXYZ> range(ref_points);

  LabelBase label(ref_points.Size());
  LabelOutOfRange(label, range, 2.0, 8.0);
  EXPECT_THAT(
    label.Get(),
    testing::ElementsAre(
      PointLabel::OutOfRange,
      PointLabel::Default,
      PointLabel::Default,
      PointLabel::Default,
      PointLabel::OutOfRange));
}
