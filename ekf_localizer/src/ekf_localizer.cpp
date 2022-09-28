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

#include "ekf_localizer/ekf_localizer.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <utility>

#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "lidar_feature_library/eigen.hpp"
#include "lidar_feature_library/ros_msg.hpp"
#include "rotationlib/quaternion.hpp"

#include "ekf_localizer/check.hpp"
#include "ekf_localizer/delay.hpp"
#include "ekf_localizer/mahalanobis.hpp"
#include "ekf_localizer/matrix_types.hpp"
#include "ekf_localizer/pose_measurement.hpp"
#include "ekf_localizer/state_transition.hpp"
#include "ekf_localizer/string.hpp"
#include "ekf_localizer/tf.hpp"
#include "ekf_localizer/warning.hpp"


inline Eigen::Vector3d createRPYfromQuaternion(
  const geometry_msgs::msg::Quaternion & orientation)
{
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  tf2::Quaternion q_tf;
  tf2::fromMsg(orientation, q_tf);
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
  return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Isometry3d MakePoseFromXYZRPY(
  const double x, const double y, const double z,
  const double roll, const double pitch, const double yaw)
{
  Eigen::Isometry3d pose;
  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() = rotationlib::RPYToQuaternionXYZ(roll, pitch, yaw).toRotationMatrix();
  return pose;
}

std::array<double, 36> EKFCovarianceToPoseMessageCovariance(const Matrix6d & P)
{
  const double p00 = P(0, 0);
  const double p01 = P(0, 1);
  const double p02 = P(0, 2);
  const double p10 = P(1, 0);
  const double p11 = P(1, 1);
  const double p12 = P(1, 2);
  const double p20 = P(2, 0);
  const double p21 = P(2, 1);
  const double p22 = P(2, 2);

  return std::array<double, 36>{
    p00, p01, 0.0, 0.0, 0.0, p02,
    p10, p11, 0.0, 0.0, 0.0, p12,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    p20, p21, 0.0, 0.0, 0.0, p22
  };
}

std::array<double, 36> EKFCovarianceToTwistMessageCovariance(const Matrix6d & P)
{
  const double p44 = P(4, 4);
  const double p45 = P(4, 5);
  const double p54 = P(5, 4);
  const double p55 = P(5, 5);

  return std::array<double, 36>{
    p44, 0.0, 0.0, 0.0, 0.0, p45,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    p54, 0.0, 0.0, 0.0, 0.0, p55
  };
}

void publishEstimateResult(
  const Eigen::MatrixXd & P,
  const rclcpp::Time & current_time,
  const std::string & pose_frame_id,
  const Eigen::Isometry3d & unbiased_pose,
  const Eigen::Vector3d & linear,
  const Eigen::Vector3d & angular,
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pub_odom_)
{
  /* publish latest pose with covariance */
  geometry_msgs::msg::PoseWithCovariance unbiased_pose_msg;
  unbiased_pose_msg.pose = MakePose(unbiased_pose);
  unbiased_pose_msg.covariance = EKFCovarianceToPoseMessageCovariance(P);

  /* publish latest twist with covariance */
  geometry_msgs::msg::TwistWithCovariance twist_msg;
  twist_msg.twist = MakeTwist(linear, angular);
  twist_msg.covariance = EKFCovarianceToTwistMessageCovariance(P);

  /* publish latest odometry */
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = current_time;
  odometry.header.frame_id = pose_frame_id;
  odometry.child_frame_id = "base_link";
  odometry.pose = unbiased_pose_msg;
  odometry.twist = twist_msg;
  pub_odom_->publish(odometry);
}

double InitYawBias(const bool enable_yaw_bias_estimation, const double initial_value)
{
  if (enable_yaw_bias_estimation) {
    return initial_value;
  }
  return 0.;
}

std::chrono::nanoseconds DoubleToNanoSeconds(const double time)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(time));
}

Vector6d InitState(const double x, const double y, const double yaw)
{
  return (Vector6d() << x, y, yaw, 0.0, 0.0, 0.0).finished();
}

Matrix6d InitCovariance(const std::array<double, 36> & initial_pose_covariance)
{
  const Matrix6d C = GetEigenCovariance(initial_pose_covariance);
  const Vector6d d = (Vector6d() << C(0, 0), C(1, 1), C(5, 5), 0.0001, 0.01, 0.01).finished();
  return d.asDiagonal();
}

EKFLocalizer::EKFLocalizer(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options),
  warning_(this),
  listener_(this),
  pub_odom_(create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 1)),
  sub_initialpose_(create_subscription<PoseWithCovarianceStamped>(
      "initialpose", 1,
      std::bind(&EKFLocalizer::callbackInitialPose, this, std::placeholders::_1))),
  sub_pose_with_cov_(create_subscription<PoseWithCovarianceStamped>(
      "in_pose_with_covariance", 1,
      std::bind(&EKFLocalizer::callbackPoseWithCovariance, this, std::placeholders::_1))),
  sub_twist_with_cov_(create_subscription<TwistWithCovarianceStamped>(
      "in_twist_with_covariance", 1,
      std::bind(&EKFLocalizer::callbackTwistWithCovariance, this, std::placeholders::_1))),
  tf_br_(std::make_shared<tf2_ros::TransformBroadcaster>(
      std::shared_ptr<rclcpp::Node>(this, [](auto) {}))),
  params(EKFParameters(this)),
  interval_(params.default_frequency_),
  yaw_bias_covariance_(
    InitYawBias(params.enable_yaw_bias_estimation, params.proc_stddev_yaw_bias_c)),
  variance_(
    params.yaw_covariance_, yaw_bias_covariance_, params.vx_covariance_, params.wz_covariance_),
  ekf_(nullptr),
  pose_measurement_(
    warning_, params.pose_frame_id_, params.extend_state_step_,
    params.pose_gate_dist_, params.pose_smoothing_steps_),
  twist_measurement_(
    warning_, params.extend_state_step_, params.twist_gate_dist_, params.twist_smoothing_steps_)
{
  const double timer_interval = ComputeInterval(params.default_frequency_);
  timer_control_ = rclcpp::create_timer(
    this, get_clock(), DoubleToNanoSeconds(timer_interval),
    std::bind(&EKFLocalizer::timerCallback, this));

  z_filter_.set_proc_stddev(1.0);
  roll_filter_.set_proc_stddev(0.1);
  pitch_filter_.set_proc_stddev(0.1);
}

/*  == Nonlinear model ==
 *
 * x_{k+1}   = x_k + vx_k * cos(yaw_k + b_k) * dt
 * y_{k+1}   = y_k + vx_k * sin(yaw_k + b_k) * dt
 * yaw_{k+1} = yaw_k + (wz_k) * dt
 * b_{k+1}   = b_k
 * vx_{k+1}  = vz_k
 * wz_{k+1}  = wz_k
 *
 * (b_k : yaw_bias_k)
 */

/*  == Linearized model ==
 *
 * A = [ 1, 0, -vx*sin(yaw+b)*dt, -vx*sin(yaw+b)*dt, cos(yaw+b)*dt,  0]
 *     [ 0, 1,  vx*cos(yaw+b)*dt,  vx*cos(yaw+b)*dt, sin(yaw+b)*dt,  0]
 *     [ 0, 0,                 1,                 0,             0, dt]
 *     [ 0, 0,                 0,                 1,             0,  0]
 *     [ 0, 0,                 0,                 0,             1,  0]
 *     [ 0, 0,                 0,                 0,             0,  1]
 */
void EKFLocalizer::timerCallback()
{
  if (ekf_ == nullptr) {
    return;
  }

  const auto maybe_dt = [&]() -> std::optional<double> {
      try {
        const rclcpp::Time current_time = this->get_clock()->now();
        const double dt = interval_.Compute(current_time.seconds());
        return std::make_optional<double>(dt);
      } catch (const std::invalid_argument & e) {
        RCLCPP_WARN(this->get_logger(), e.what());
        return std::nullopt;
      }
    }();

  if (!maybe_dt.has_value()) {
    return;
  }

  const double dt = maybe_dt.value();

  const Vector6d x_curr = ekf_->getLatestX();  // current state
  const Vector6d x_next = predictNextState(x_curr, dt);
  const Matrix6d A = createStateTransitionMatrix(x_curr, dt);
  const Matrix6d Q = processNoiseCovariance(variance_.TimeScaledVariances(dt));

  ekf_->predict(x_next, A, Q);

  pose_measurement_.Update(ekf_, this->now(), dt);
  twist_measurement_.Update(ekf_, this->now(), dt);

  const Vector6d x_est = ekf_->getLatestX();
  const double x = x_est(0);
  const double y = x_est(1);
  const double biased_yaw = x_est(2);
  const double yaw_bias = x_est(3);
  const double vx = x_est(4);
  const double wz = x_est(5);

  const double z = z_filter_.get_x();
  const double roll = roll_filter_.get_x();
  const double pitch = pitch_filter_.get_x();
  const double yaw = biased_yaw + yaw_bias;

  const Eigen::Isometry3d unbiased_pose = MakePoseFromXYZRPY(x, y, z, roll, pitch, yaw);

  const Eigen::Vector3d linear(vx, 0, 0);
  const Eigen::Vector3d angular(0, 0, wz);

  tf_br_->sendTransform(
    MakeTransformStamped(unbiased_pose, this->now(), params.pose_frame_id_, "base_link"));

  publishEstimateResult(
    ekf_->getLatestP(), this->now(), params.pose_frame_id_,
    unbiased_pose, linear, angular, pub_odom_);
}

void EKFLocalizer::callbackInitialPose(PoseWithCovarianceStamped::SharedPtr initialpose)
{
  const auto maybe_transform = listener_.LookupTransform(
    EraseBeginSlash(params.pose_frame_id_),
    EraseBeginSlash(initialpose->header.frame_id));
  if (!maybe_transform.has_value()) {
    RCLCPP_ERROR(
      get_logger(), "[EKF] TF transform failed. parent = %s, child = %s",
      params.pose_frame_id_.c_str(), initialpose->header.frame_id.c_str());
  }

  const Eigen::Vector3d initial_position = ToVector3d(initialpose->pose.pose.position);
  const Eigen::Vector3d translation = ToVector3d(maybe_transform->transform.translation);
  const Eigen::Vector3d t = initial_position + translation;
  const double initial_yaw = tf2::getYaw(initialpose->pose.pose.orientation);
  const double yaw = tf2::getYaw(maybe_transform->transform.rotation);

  const Vector6d x = InitState(t(0), t(1), initial_yaw + yaw);
  const Matrix6d P = InitCovariance(initialpose->pose.covariance);
  ekf_.reset(new TimeDelayKalmanFilter(x, P, params.extend_state_step_));

  updateSimple1DFilters(*initialpose);

  pose_measurement_.Clear();
  twist_measurement_.Clear();
}

void EKFLocalizer::callbackPoseWithCovariance(PoseWithCovarianceStamped::SharedPtr msg)
{
  pose_measurement_.Push(msg);

  updateSimple1DFilters(*msg);
}

void EKFLocalizer::callbackTwistWithCovariance(TwistWithCovarianceStamped::SharedPtr msg)
{
  twist_measurement_.Push(msg);
}

void EKFLocalizer::updateSimple1DFilters(const PoseWithCovarianceStamped & pose)
{
  const double z = pose.pose.pose.position.z;

  const Eigen::Vector3d rpy = createRPYfromQuaternion(pose.pose.pose.orientation);

  const Matrix6d covariance = GetEigenCovariance(pose.pose.covariance);
  const double z_stddev = std::sqrt(covariance(2, 2));
  const double roll_stddev = std::sqrt(covariance(3, 3));
  const double pitch_stddev = std::sqrt(covariance(4, 4));

  z_filter_.update(z, z_stddev, pose.header.stamp);
  roll_filter_.update(rpy(0), roll_stddev, pose.header.stamp);
  pitch_filter_.update(rpy(1), pitch_stddev, pose.header.stamp);
}
