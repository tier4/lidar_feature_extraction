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

#ifndef LIDAR_FEATURE_LOCALIZATION__LOCALIZER_HPP_
#define LIDAR_FEATURE_LOCALIZATION__LOCALIZER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tuple>

#include "lidar_feature_localization/loam.hpp"
#include "lidar_feature_localization/optimizer.hpp"

#include "lidar_feature_library/convert_point_cloud_type.hpp"
#include "lidar_feature_library/point_type.hpp"

using ArgumentType = std::tuple<
  pcl::PointCloud<pcl::PointXYZ>::Ptr,
  pcl::PointCloud<pcl::PointXYZ>::Ptr>;

class Localizer
{
public:
  Localizer(
    const typename pcl::PointCloud<PointXYZIR>::Ptr & edge_map,
    const typename pcl::PointCloud<PointXYZIR>::Ptr & surface_map)
  : edge_map_(ToPointXYZ<PointXYZIR>(edge_map)),
    surface_map_(ToPointXYZ<PointXYZIR>(surface_map)),
    is_initialized_(false),
    pose_(Eigen::Isometry3d::Identity())
  {
  }

  void Init(const Eigen::Isometry3d & initial_pose)
  {
    pose_ = initial_pose;
    is_initialized_ = true;
  }

  bool Run(
    const pcl::PointCloud<PointXYZIR>::Ptr & edge,
    const pcl::PointCloud<PointXYZIR>::Ptr & surface)
  {
    const pcl::PointCloud<pcl::PointXYZ>::Ptr edge_xyz = ToPointXYZ<PointXYZIR>(edge);
    const pcl::PointCloud<pcl::PointXYZ>::Ptr surface_xyz = ToPointXYZ<PointXYZIR>(surface);
    const auto [pose, success] = this->Run(edge_xyz, surface_xyz, pose_);

    pose_ = pose;
    return success;
  }

  Eigen::Isometry3d Get() const
  {
    return pose_;
  }

  bool IsInitialized() const
  {
    return is_initialized_;
  }

private:
  std::tuple<Eigen::Isometry3d, bool> Run(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & edge,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & surface,
    const Eigen::Isometry3d & pose) const
  {
    const LOAMOptimizationProblem problem(edge_map_, surface_map_);

    if (problem.IsDegenerate(edge, surface, pose)) {
      RCLCPP_WARN(
        rclcpp::get_logger("lidar_feature_localization"),
        "The optimization problem is degenerate. Pose not optimized");
      return std::make_tuple(pose, false);
    }

    const Optimizer<LOAMOptimizationProblem, ArgumentType> optimizer(problem);
    const Eigen::Isometry3d new_pose = optimizer.Run(std::make_tuple(edge, surface), pose);
    return std::make_tuple(new_pose, true);
  }

  const typename pcl::PointCloud<pcl::PointXYZ>::Ptr edge_map_;
  const typename pcl::PointCloud<pcl::PointXYZ>::Ptr surface_map_;

  bool is_initialized_;
  Eigen::Isometry3d pose_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__LOCALIZER_HPP_
