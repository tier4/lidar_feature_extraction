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

#include <rclcpp/rclcpp.hpp>

#include <rcpputils/filesystem_helper.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>

#include <limits>
#include <memory>
#include <string>
#include <tuple>

#include "lidar_feature_localization/localizer.hpp"
#include "lidar_feature_localization/matrix_type.hpp"
#include "lidar_feature_localization/posevec.hpp"
#include "lidar_feature_localization/subscriber.hpp"

using PointType = pcl::PointXYZ;
using PointToVector = PointXYZToVector;
using Subscriber = LocalizationSubscriber<Localizer<PointToVector, PointType>, PointType>;

const std::string edge_map_path = "maps/edge.pcd";
const std::string surface_map_path = "maps/surface.pcd";

constexpr int max_iter = 40;

bool CheckMapPathExists(const std::string & map_path)
{
  bool exists = rcpputils::fs::exists(map_path);
  if (!exists) {
    RCLCPP_ERROR(
      rclcpp::get_logger("lidar_feature_localization"),
      "Map %s does not exist!", map_path.c_str());
  }
  return exists;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (!CheckMapPathExists(edge_map_path)) {
    return -1;
  }

  if (!CheckMapPathExists(surface_map_path)) {
    return -1;
  }

  pcl::PointCloud<PointType>::Ptr edge_map(new pcl::PointCloud<PointType>());
  pcl::io::loadPCDFile(edge_map_path, *edge_map);

  pcl::PointCloud<PointType>::Ptr surface_map(new pcl::PointCloud<PointType>());
  pcl::io::loadPCDFile(surface_map_path, *surface_map);

  Localizer<PointToVector, PointType> localizer(edge_map, surface_map, max_iter);
  rclcpp::spin(std::make_shared<Subscriber>(localizer));
  rclcpp::shutdown();
  return 0;
}
