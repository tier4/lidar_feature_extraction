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

#include "ekf_localizer/tf.hpp"
#include "ekf_localizer/update_interval.hpp"
#include "ekf_localizer/warning.hpp"

using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;

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

template<typename Message>
class AgedMessageQueue
{
public:
  explicit AgedMessageQueue(const int max_age)
  : max_age_(max_age)
  {
  }

  size_t size()
  {
    return msgs_.size();
  }

  void push(const Message & msg)
  {
    msgs_.push(msg);
    ages_.push(0);
  }

  Message pop()
  {
    const auto msg = msgs_.front();
    const int age = ages_.front() + 1;
    msgs_.pop();
    ages_.pop();

    if (age < max_age_) {
      msgs_.push(msg);
      ages_.push(age);
    }

    return msg;
  }

  void clear()
  {
    msgs_ = std::queue<Message>();
    ages_ = std::queue<int>();
  }

private:
  const int max_age_;
  std::queue<Message> msgs_;
  std::queue<int> ages_;
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
  }
  void init(const double init_obs, const double obs_stddev, const rclcpp::Time time)
  {
    x_ = init_obs;
    stddev_ = obs_stddev;
    latest_time_ = time;
    initialized_ = true;
    return;
  }
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
  }
  void set_proc_stddev(const double proc_stddev) {proc_stddev_x_c_ = proc_stddev;}
  double get_x() {return x_;}

private:
  bool initialized_;
  double x_;
  double stddev_;
  double proc_stddev_x_c_;
  rclcpp::Time latest_time_;
};

inline double TimeScaledVariance(const double stddev, const double dt)
{
  return stddev * stddev * dt * dt;
}

class DefaultVariance
{
public:
  DefaultVariance(
    const double yaw_covariance,
    const double yaw_bias_covariance,
    const double vx_covariance,
    const double wz_covariance)
  : yaw_covariance_(yaw_covariance),
    yaw_bias_covariance_(yaw_bias_covariance),
    vx_covariance_(vx_covariance),
    wz_covariance_(wz_covariance)
  {
  }

  Eigen::Vector4d TimeScaledVariances(const double dt) const
  {
    const double yaw_variance = TimeScaledVariance(yaw_covariance_, dt);
    const double yaw_bias_variance = TimeScaledVariance(yaw_bias_covariance_, dt);
    const double vx_variance = TimeScaledVariance(vx_covariance_, dt);
    const double wz_variance = TimeScaledVariance(wz_covariance_, dt);
    return Eigen::Vector4d(yaw_variance, yaw_bias_variance, vx_variance, wz_variance);
  }

private:
  const double yaw_covariance_;
  const double yaw_bias_covariance_;
  const double vx_covariance_;
  const double wz_covariance_;
};

class EKFLocalizer : public rclcpp::Node
{
public:
  EKFLocalizer(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  const Warning warning_;

  const TransformListener listener_;

  //!< @brief estimated ekf odometry publisher
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  //!< @brief ekf estimated yaw bias publisher
  const rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_biased_pose_;
  //!< @brief initial pose subscriber
  const rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  //!< @brief measurement pose with covariance subscriber
  const rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_;
  //!< @brief measurement twist with covariance subscriber
  const rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr
    sub_twist_with_cov_;
  //!< @brief time for ekf calculation callback
  rclcpp::TimerBase::SharedPtr timer_control_;
  //!< @brief last predict time
  std::optional<rclcpp::Time> last_predict_time_;

  //!< @brief tf broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  //!< @brief  extended kalman filter instance.
  Simple1DFilter z_filter_;
  Simple1DFilter roll_filter_;
  Simple1DFilter pitch_filter_;

  const double default_frequency_;

  UpdateInterval interval_;

  /* parameters */
  const double tf_rate_;                   //!< @brief  tf publish rate
  const bool enable_yaw_bias_estimation_;  //!< @brief for LiDAR mount error.
  //!< if true,publish /estimate_yaw_bias
  const int extend_state_step_;  //!< @brief  for time delay compensation

  const std::string pose_frame_id_;

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

  const DefaultVariance variance_;

  TimeDelayKalmanFilter ekf_;

  AgedMessageQueue<PoseWithCovarianceStamped::SharedPtr> pose_messages_;
  AgedMessageQueue<TwistWithCovarianceStamped::SharedPtr> twist_messages_;

  std::array<double, 36ul> current_pose_covariance_;
  std::array<double, 36ul> current_twist_covariance_;

  /**
   * @brief computes update & prediction of EKF for each ekf_dt_[s] time
   */
  void timerCallback();

  /**
   * @brief set poseWithCovariance measurement
   */
  void callbackPoseWithCovariance(PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief set twistWithCovariance measurement
   */
  void callbackTwistWithCovariance(TwistWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief set initial_pose to current EKF pose
   */
  void callbackInitialPose(PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief initialization of EKF
   */
  void initEKF();

  /**
   * @brief update predict frequency
   */
  void updatePredictFrequency();

  void updateSimple1DFilters(const PoseWithCovarianceStamped & pose);

  friend class EKFLocalizerTestSuite;  // for test code
};

#endif  // EKF_LOCALIZER__EKF_LOCALIZER_HPP_
