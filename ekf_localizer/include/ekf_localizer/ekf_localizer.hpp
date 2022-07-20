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

#ifndef EKF_LOCALIZER__EKF_LOCALIZER_HPP_
#define EKF_LOCALIZER__EKF_LOCALIZER_HPP_

#include <kalman_filter/kalman_filter.hpp>
#include <kalman_filter/time_delay_kalman_filter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "ekf_localizer/warning.hpp"


// Noramlizes the yaw angle so that it fits in the range (-pi, pi)
/**
* @brief normalize yaw angle
* @param yaw yaw angle
* @return normalized yaw
*/
inline double normalizeYaw(const double & yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

inline double SquaredMahalanobis(
  const Eigen::VectorXd & x,
  const Eigen::VectorXd & y,
  const Eigen::MatrixXd & C)
{
  const Eigen::VectorXd d = x - y;
  return d.dot(C.inverse() * d);
}

/**
 * @brief check whether a measurement value falls within the mahalanobis distance threshold
 * @param dist_max mahalanobis distance threshold
 * @param estimated current estimated state
 * @param measured measured state
 * @param estimated_cov current estimation covariance
 * @return whether it falls within the mahalanobis distance threshold
 */
inline bool mahalanobisGate(
  const double & dist_max, const Eigen::MatrixXd & x, const Eigen::MatrixXd & obj_x,
  const Eigen::MatrixXd & cov)
{
  const double squared_distance = SquaredMahalanobis(x, obj_x, cov);
  if (squared_distance > dist_max * dist_max) {
    return false;
  }

  return true;
}

struct PoseInfo
{
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose;
  int counter;
};

struct TwistInfo
{
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist;
  int counter;
};

class Simple1DFilter
{
public:
  Simple1DFilter()
  {
    initialized_ = false;
    x_ = 0;
    stddev_ = 1e9;
    proc_stddev_x_c_ = 0.0;
    return;
  };
  void init(const double init_obs, const double obs_stddev, const rclcpp::Time time)
  {
    x_ = init_obs;
    stddev_ = obs_stddev;
    latest_time_ = time;
    initialized_ = true;
    return;
  };
  void update(const double obs, const double obs_stddev, const rclcpp::Time time)
  {
    if (!initialized_) {
      init(obs, obs_stddev, time);
      return;
    }

    // Prediction step (current stddev_)
    double dt = (time - latest_time_).seconds();
    double proc_stddev_x_d = proc_stddev_x_c_ * dt;
    stddev_ = std::sqrt(stddev_ * stddev_ + proc_stddev_x_d * proc_stddev_x_d);

    // Update step
    double kalman_gain = stddev_ * stddev_ / (stddev_ * stddev_ + obs_stddev * obs_stddev);
    x_ = x_ + kalman_gain * (obs - x_);
    stddev_ = std::sqrt(1 - kalman_gain) * stddev_;

    latest_time_ = time;
    return;
  };
  void set_proc_stddev(const double proc_stddev) { proc_stddev_x_c_ = proc_stddev; }
  double get_x() { return x_; }

private:
  bool initialized_;
  double x_;
  double stddev_;
  double proc_stddev_x_c_;
  rclcpp::Time latest_time_;
};

class EKFLocalizer : public rclcpp::Node
{
public:
  EKFLocalizer(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  const Warning warning_;

  //!< @brief ekf estimated pose publisher
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  //!< @brief estimated ekf pose with covariance publisher
  const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  //!< @brief estimated ekf odometry publisher
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  //!< @brief ekf estimated twist publisher
  const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  //!< @brief ekf estimated twist with covariance publisher
  const rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;
  //!< @brief debug measurement pose publisher
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_measured_pose_;
  //!< @brief ekf estimated yaw bias publisher
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_no_yawbias_;
  //!< @brief ekf estimated yaw bias publisher
  const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pub_pose_cov_no_yawbias_;
  //!< @brief initial pose subscriber
  const rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  //!< @brief measurement pose with covariance subscriber
  const rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_;
  //!< @brief measurement twist with covariance subscriber
  const rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    sub_twist_with_cov_;
  //!< @brief time for ekf calculation callback
  rclcpp::TimerBase::SharedPtr timer_control_;
  //!< @brief last predict time
  std::shared_ptr<const rclcpp::Time> last_predict_time_;

  //!< @brief timer to send transform
  rclcpp::TimerBase::SharedPtr timer_tf_;
  //!< @brief tf broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  //!< @brief  extended kalman filter instance.
  TimeDelayKalmanFilter ekf_;
  Simple1DFilter z_filter_;
  Simple1DFilter roll_filter_;
  Simple1DFilter pitch_filter_;

  /* parameters */
  const bool show_debug_info_;
  double ekf_rate_;                  //!< @brief  EKF predict rate
  double ekf_dt_;                    //!< @brief  = 1 / ekf_rate_
  const double tf_rate_;                   //!< @brief  tf publish rate
  const bool enable_yaw_bias_estimation_;  //!< @brief for LiDAR mount error.
                                     //!< if true,publish /estimate_yaw_bias
  const int extend_state_step_;  //!< @brief  for time delay compensation

  const std::string pose_frame_id_;

  const int dim_x_;              //!< @brief  dimension of EKF state
  const int pose_smoothing_steps_;

  const double pose_additional_delay_;    //!< @brief  compensated pose delay time =
                                          //!< (pose.header.stamp - now) + additional_delay [s]
  //!< @brief  the mahalanobis distance threshold to ignore pose measurement
  const double pose_gate_dist_;

  const double twist_additional_delay_;  //!< @brief  compensated delay = (twist.header.stamp - now)
                                   //!< + additional_delay [s]
  //!< @brief  measurement is ignored if the mahalanobis distance is larger than this value.
  const double twist_gate_dist_;

  const int twist_smoothing_steps_;

  /* process noise standard deviation */
  const double yaw_covariance_;       //!< @brief  yaw process noise
  const double yaw_bias_covariance_;  //!< @brief  yaw bias process noise
  const double vx_covariance_;        //!< @brief  vx process noise
  const double wz_covariance_;        //!< @brief  wz process noise

  /* process noise variance for discrete model */
  Eigen::Vector4d variances_;

  /* for model prediction */
  std::queue<TwistInfo> current_twist_info_queue_;    //!< @brief current measured pose
  std::queue<PoseInfo> current_pose_info_queue_;      //!< @brief current measured pose
  geometry_msgs::msg::PoseStamped current_ekf_pose_;  //!< @brief current estimated pose
  std::array<double, 36ul> current_pose_covariance_;
  std::array<double, 36ul> current_twist_covariance_;

  /**
   * @brief computes update & prediction of EKF for each ekf_dt_[s] time
   */
  void timerCallback();

  /**
   * @brief publish tf for tf_rate [Hz]
   */
  void timerTFCallback();

  /**
   * @brief set poseWithCovariance measurement
   */
  void callbackPoseWithCovariance(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief set twistWithCovariance measurement
   */
  void callbackTwistWithCovariance(geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief set initial_pose to current EKF pose
   */
  void callbackInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief initialization of EKF
   */
  void initEKF();

  /**
   * @brief update predict frequency
   */
  void updatePredictFrequency();

  /**
   * @brief compute EKF update with pose measurement
   * @param pose measurement value
   */
  void measurementUpdatePose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  /**
   * @brief compute EKF update with pose measurement
   * @param twist measurement value
   */
  void measurementUpdateTwist(const geometry_msgs::msg::TwistWithCovarianceStamped & twist);

  /**
   * @brief get transform from frame_id
   */
  bool getTransformFromTF(
    std::string parent_frame, std::string child_frame,
    geometry_msgs::msg::TransformStamped & transform);

  void updateSimple1DFilters(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);

  friend class EKFLocalizerTestSuite;  // for test code
};

#endif  // EKF_LOCALIZER__EKF_LOCALIZER_HPP_
