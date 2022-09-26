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
#include "ekf_localizer/mahalanobis.hpp"
#include "ekf_localizer/measurement.hpp"
#include "ekf_localizer/state_transition.hpp"
#include "ekf_localizer/string.hpp"
#include "ekf_localizer/tf.hpp"
#include "ekf_localizer/warning.hpp"

using Vector6d = Eigen::Matrix<double, 6, 1>;

constexpr int DIM_X = 6;


// Revival of tf::createQuaternionFromRPY
// https://answers.ros.org/question/304397/recommended-way-to-construct-quaternion-from-rollpitchyaw-with-tf2/
inline geometry_msgs::msg::Quaternion createQuaternionFromRPY(
  const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

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

std::unique_ptr<TimeDelayKalmanFilter> InitEKF(
  const int extend_state_step_, const double yaw_bias_variance)
{
  Matrix6d P = Matrix6d::Identity() * 1.0E15;    // for x & y
  P(2, 2) = 50.0;                                // for yaw
  P(3, 3) = yaw_bias_variance;                   // for yaw bias
  P(4, 4) = 1000.0;                              // for vx
  P(5, 5) = 50.0;                                // for wz

  return std::make_unique<TimeDelayKalmanFilter>(Vector6d::Zero(), P, extend_state_step_);
}

Eigen::Vector3d PoseMeasurementVector(const geometry_msgs::msg::Pose & pose)
{
  const double yaw = tf2::getYaw(pose.orientation);
  return Eigen::Vector3d(pose.position.x, pose.position.y, normalizeYaw(yaw));
}

Eigen::Vector3d GetPoseState(const Vector6d & x)
{
  return x.head(3);
}

Eigen::Vector2d GetTwistState(const Vector6d & x)
{
  return x.tail(2);
}

Eigen::Matrix3d PoseCovariance(const Eigen::MatrixXd & P)
{
  return P.block(0, 0, 3, 3);
}

Eigen::Matrix2d TwistCovariance(const Eigen::MatrixXd & P)
{
  return P.block(4, 4, 2, 2);
}

Eigen::Vector2d TwistMeasurementVector(const geometry_msgs::msg::Twist & twist)
{
  return Eigen::Vector2d(twist.linear.x, twist.angular.z);
}

double ComputeDelayTime(
  const rclcpp::Time & current_time,
  const rclcpp::Time & message_stamp,
  const double additional_delay)
{
  return (current_time - message_stamp).seconds() + additional_delay;
}

int ComputeDelayStep(const double delay_time, const double dt)
{
  return std::roundf(std::max(delay_time, 0.) / dt);
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
  last_predict_time_(std::nullopt),
  tf_br_(std::make_shared<tf2_ros::TransformBroadcaster>(
      std::shared_ptr<rclcpp::Node>(this, [](auto) {}))),
  default_frequency_(declare_parameter("predict_frequency", 50.0)),
  interval_(default_frequency_),
  tf_rate_(declare_parameter("tf_rate", 10.0)),
  enable_yaw_bias_estimation_(declare_parameter("enable_yaw_bias_estimation", true)),
  extend_state_step_(declare_parameter("extend_state_step", 50)),
  pose_frame_id_(declare_parameter("pose_frame_id", std::string("map"))),
  pose_smoothing_steps_(declare_parameter("pose_smoothing_steps", 5)),
  pose_additional_delay_(declare_parameter("pose_additional_delay", 0.0)),
  pose_gate_dist_(declare_parameter("pose_gate_dist", 10000.0)),
  twist_additional_delay_(declare_parameter("twist_additional_delay", 0.0)),
  twist_gate_dist_(declare_parameter("twist_gate_dist", 10000.0)),
  twist_smoothing_steps_(declare_parameter("twist_smoothing_steps", 2)),
  yaw_covariance_(declare_parameter("proc_stddev_yaw_c", 0.005)),
  yaw_bias_covariance_(
    InitYawBias(enable_yaw_bias_estimation_, declare_parameter("proc_stddev_yaw_bias_c", 0.001))),
  vx_covariance_(declare_parameter("proc_stddev_vx_c", 5.0)),
  wz_covariance_(declare_parameter("proc_stddev_wz_c", 1.0)),
  variance_(yaw_covariance_, yaw_bias_covariance_, vx_covariance_, wz_covariance_),
  pose_messages_(pose_smoothing_steps_),
  twist_messages_(twist_smoothing_steps_)
{
  const double timer_interval = ComputeInterval(default_frequency_);

  ekf_ = InitEKF(extend_state_step_, TimeScaledVariance(yaw_bias_covariance_, timer_interval));

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

  ekf_->predictWithDelay(x_next, A, Q);

  /* pose measurement update */
  for (size_t i = 0; i < pose_messages_.size(); ++i) {
    const auto pose = pose_messages_.pop();

    CheckFrameId(warning_, pose->header.frame_id, pose_frame_id_);

    const double delay_time = ComputeDelayTime(
      this->now(), pose->header.stamp, pose_additional_delay_);
    CheckDelayTime(warning_, delay_time);

    const int delay_step = ComputeDelayStep(delay_time, dt);
    if (!CheckDelayStep(warning_, delay_step, extend_state_step_)) {
      continue;
    }

    const Eigen::Vector3d y = PoseMeasurementVector(pose->pose.pose);

    if (!CheckMeasurementMatrixNanInf(warning_, y)) {
      continue;
    }

    const Eigen::Vector3d y_ekf = GetPoseState(ekf_->getX(delay_step));
    const Eigen::Matrix3d P_y = PoseCovariance(ekf_->getLatestP());

    if (!CheckMahalanobisGate(warning_, pose_gate_dist_, y_ekf, y, P_y)) {
      continue;
    }

    const Eigen::Matrix<double, 3, 6> C = PoseMeasurementMatrix();
    const Eigen::Matrix3d R = PoseMeasurementCovariance(
      pose->pose.covariance, pose_smoothing_steps_);

    ekf_->updateWithDelay(y, C, R, delay_step);
  }

  /* twist measurement update */
  for (size_t i = 0; i < twist_messages_.size(); ++i) {
    const auto twist = twist_messages_.pop();

    CheckFrameId(warning_, twist->header.frame_id, "base_link");

    const double delay_time = ComputeDelayTime(
      this->now(), twist->header.stamp, twist_additional_delay_);
    CheckDelayTime(warning_, delay_time);

    const int delay_step = ComputeDelayStep(delay_time, dt);
    if (!CheckDelayStep(warning_, delay_step, extend_state_step_)) {
      continue;
    }

    const Eigen::Vector2d y = TwistMeasurementVector(twist->twist.twist);

    if (!CheckMeasurementMatrixNanInf(warning_, y)) {
      continue;
    }

    const Eigen::Vector2d y_ekf = GetTwistState(ekf_->getX(delay_step));
    const Eigen::Matrix2d P_y = TwistCovariance(ekf_->getLatestP());

    if (!CheckMahalanobisGate(warning_, twist_gate_dist_, y_ekf, y, P_y)) {
      continue;
    }

    const Eigen::Matrix<double, 2, 6> C = TwistMeasurementMatrix();
    const Eigen::Matrix2d R = TwistMeasurementCovariance(
      twist->twist.covariance, twist_smoothing_steps_);
    ekf_->updateWithDelay(y, C, R, delay_step);
  }

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
    MakeTransformStamped(unbiased_pose, this->now(), pose_frame_id_, "base_link"));

  /* publish ekf result */
  publishEstimateResult(
    ekf_->getLatestP(), this->now(), pose_frame_id_, unbiased_pose, linear, angular, pub_odom_);
}

/*
 * callbackInitialPose
 */
void EKFLocalizer::callbackInitialPose(PoseWithCovarianceStamped::SharedPtr initialpose)
{
  geometry_msgs::msg::TransformStamped transform;

  const auto maybe_transform = listener_.LookupTransform(
    EraseBeginSlash(pose_frame_id_),
    EraseBeginSlash(initialpose->header.frame_id));
  if (!maybe_transform.has_value()) {
    RCLCPP_ERROR(
      get_logger(), "[EKF] TF transform failed. parent = %s, child = %s", pose_frame_id_.c_str(),
      initialpose->header.frame_id.c_str());
  }

  // TODO(mitsudome-r) need mutex

  const Eigen::Vector3d initial_position = ToVector3d(initialpose->pose.pose.position);
  const Eigen::Vector3d translation = ToVector3d(maybe_transform->transform.translation);
  const Eigen::Vector3d t = initial_position + translation;
  const double initial_yaw = tf2::getYaw(initialpose->pose.pose.orientation);
  const double yaw = tf2::getYaw(maybe_transform->transform.rotation);

  const Vector6d x = (Vector6d() << t(0), t(1), initial_yaw + yaw, 0.0, 0.0, 0.0).finished();

  const Matrix6d C = GetEigenCovariance(initialpose->pose.covariance);
  const Vector6d d = (Vector6d() << C(0, 0), C(1, 1), C(5, 5), 0.0001, 0.01, 0.01).finished();
  const Matrix6d P = d.asDiagonal();

  ekf_.reset(new TimeDelayKalmanFilter(x, P, extend_state_step_));

  updateSimple1DFilters(*initialpose);

  pose_messages_.clear();
}

/*
 * callbackPoseWithCovariance
 */
void EKFLocalizer::callbackPoseWithCovariance(PoseWithCovarianceStamped::SharedPtr msg)
{
  pose_messages_.push(msg);

  updateSimple1DFilters(*msg);
}

/*
 * callbackTwistWithCovariance
 */
void EKFLocalizer::callbackTwistWithCovariance(TwistWithCovarianceStamped::SharedPtr msg)
{
  twist_messages_.push(msg);
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
