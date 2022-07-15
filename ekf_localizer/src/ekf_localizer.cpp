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

#define FMT_HEADER_ONLY
#include <fmt/format.h>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <utility>

#include "lidar_feature_library/eigen.hpp"
#include "lidar_feature_library/ros_msg.hpp"
#include "rotationlib/quaternion.hpp"


// clang-format off
#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

using Vector6d = Eigen::Matrix<double, 6, 1>;

// clang-format on

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

void publishEstimateResult(
  const int dim_x_,
  const TimeDelayKalmanFilter & ekf_,
  const rclcpp::Time & current_time,
  const geometry_msgs::msg::PoseStamped & current_ekf_pose_,
  const geometry_msgs::msg::PoseStamped & current_ekf_pose_no_yawbias_,
  const geometry_msgs::msg::TwistStamped & current_ekf_twist_,
  const std::queue<PoseInfo> & current_pose_info_queue_,
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub_pose_,
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub_pose_no_yawbias_,
  const rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr & pub_twist_cov_,
  const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr & pub_pose_cov_,
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pub_odom_,
  const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr & pub_pose_cov_no_yawbias_,
  const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr & pub_twist_,
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub_measured_pose_)
{
  const Eigen::MatrixXd X = ekf_.getLatestX();
  const Eigen::MatrixXd P = ekf_.getLatestP();

  /* publish latest pose */
  pub_pose_->publish(current_ekf_pose_);
  pub_pose_no_yawbias_->publish(current_ekf_pose_no_yawbias_);

  /* publish latest pose with covariance */
  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = current_time;
  pose_cov.header.frame_id = current_ekf_pose_.header.frame_id;
  pose_cov.pose.pose = current_ekf_pose_.pose;

  Eigen::Map<RowMatrix6d> pose_covariance(pose_cov.pose.covariance.data(), 6, 6);
  pose_covariance(0, 0) = P(0, 0);
  pose_covariance(0, 1) = P(0, 1);
  pose_covariance(0, 5) = P(0, 2);
  pose_covariance(1, 0) = P(1, 0);
  pose_covariance(1, 1) = P(1, 1);
  pose_covariance(1, 5) = P(1, 2);
  pose_covariance(5, 0) = P(2, 0);
  pose_covariance(5, 1) = P(2, 1);
  pose_covariance(5, 5) = P(2, 2);
  pub_pose_cov_->publish(pose_cov);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_no_yawbias = pose_cov;
  pose_cov_no_yawbias.pose.pose = current_ekf_pose_no_yawbias_.pose;
  pub_pose_cov_no_yawbias_->publish(pose_cov_no_yawbias);

  /* publish latest twist */
  pub_twist_->publish(current_ekf_twist_);

  /* publish latest twist with covariance */
  geometry_msgs::msg::TwistWithCovarianceStamped twist_cov;
  twist_cov.header.stamp = current_time;
  twist_cov.header.frame_id = current_ekf_twist_.header.frame_id;
  twist_cov.twist.twist = current_ekf_twist_.twist;

  Eigen::Map<RowMatrix6d> twist_covariance(twist_cov.twist.covariance.data(), 6, 6);
  twist_covariance(0, 0) = P(4, 4);
  twist_covariance(0, 5) = P(4, 5);
  twist_covariance(5, 0) = P(5, 4);
  twist_covariance(5, 5) = P(5, 5);
  pub_twist_cov_->publish(twist_cov);

  /* publish latest odometry */
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = current_time;
  odometry.header.frame_id = current_ekf_pose_.header.frame_id;
  odometry.child_frame_id = "base_link";
  odometry.pose = pose_cov.pose;
  odometry.twist = twist_cov.twist;
  pub_odom_->publish(odometry);

  /* debug measured pose */
  if (!current_pose_info_queue_.empty()) {
    geometry_msgs::msg::PoseStamped p;
    p.pose = current_pose_info_queue_.back().pose->pose.pose;
    p.header.stamp = current_time;
    pub_measured_pose_->publish(p);
  }

  /* debug publish */
  double pose_yaw = 0.0;
  if (!current_pose_info_queue_.empty()) {
    pose_yaw = tf2::getYaw(current_pose_info_queue_.back().pose->pose.pose.orientation);
  }
}

EKFLocalizer::EKFLocalizer(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options),
  warning_(this),
  show_debug_info_(declare_parameter("show_debug_info", false)),
  ekf_rate_(declare_parameter("predict_frequency", 50.0)),
  ekf_dt_(1.0 / std::max(ekf_rate_, 0.1)),
  tf_rate_(declare_parameter("tf_rate", 10.0)),
  enable_yaw_bias_estimation_(declare_parameter("enable_yaw_bias_estimation", true)),
  extend_state_step_(declare_parameter("extend_state_step", 50)),
  pose_frame_id_(declare_parameter("pose_frame_id", std::string("map"))),
  dim_x_(6 /* x, y, yaw, yaw_bias, vx, wz */),
  pose_smoothing_steps_(declare_parameter("pose_smoothing_steps", 5)),
  tf_br_(std::make_shared<tf2_ros::TransformBroadcaster>(
    std::shared_ptr<rclcpp::Node>(this, [](auto) {}))),
  pose_additional_delay_(declare_parameter("pose_additional_delay", 0.0)),
  pose_gate_dist_(declare_parameter("pose_gate_dist", 10000.0)),
  twist_additional_delay_(declare_parameter("twist_additional_delay", 0.0)),
  twist_gate_dist_(declare_parameter("twist_gate_dist", 10000.0)),
  twist_smoothing_steps_(declare_parameter("twist_smoothing_steps", 2)),
  proc_stddev_yaw_c_(declare_parameter("proc_stddev_yaw_c", 0.005)),
  proc_stddev_yaw_bias_c_(declare_parameter("proc_stddev_yaw_bias_c", 0.001)),
  proc_stddev_vx_c_(declare_parameter("proc_stddev_vx_c", 5.0)),
  proc_stddev_wz_c_(declare_parameter("proc_stddev_wz_c", 1.0))
{

  /* process noise */
  if (!enable_yaw_bias_estimation_) {
    proc_stddev_yaw_bias_c_ = 0.0;
  }

  /* convert to continuous to discrete */
  proc_cov_vx_d_ = std::pow(proc_stddev_vx_c_ * ekf_dt_, 2.0);
  proc_cov_wz_d_ = std::pow(proc_stddev_wz_c_ * ekf_dt_, 2.0);
  proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c_ * ekf_dt_, 2.0);
  proc_cov_yaw_bias_d_ = std::pow(proc_stddev_yaw_bias_c_ * ekf_dt_, 2.0);

  /* initialize ros system */
  const auto period_control_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(ekf_dt_));
  timer_control_ = rclcpp::create_timer(
    this, get_clock(), period_control_ns, std::bind(&EKFLocalizer::timerCallback, this));

  const auto period_tf_ns = rclcpp::Rate(tf_rate_).period();
  timer_tf_ = rclcpp::create_timer(
    this, get_clock(), period_tf_ns, std::bind(&EKFLocalizer::timerTFCallback, this));

  pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose", 1);
  pub_pose_cov_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ekf_pose_with_covariance", 1);
  pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 1);
  pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("ekf_twist", 1);
  pub_twist_cov_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "ekf_twist_with_covariance", 1);
  pub_pose_no_yawbias_ =
    create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose_without_yawbias", 1);
  pub_pose_cov_no_yawbias_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance_without_yawbias", 1);
  sub_initialpose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1,
    std::bind(&EKFLocalizer::callbackInitialPose, this, std::placeholders::_1));
  sub_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", 1,
    std::bind(&EKFLocalizer::callbackPoseWithCovariance, this, std::placeholders::_1));
  sub_twist_with_cov_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance", 1,
    std::bind(&EKFLocalizer::callbackTwistWithCovariance, this, std::placeholders::_1));

  initEKF();

  z_filter_.set_proc_stddev(1.0);
  roll_filter_.set_proc_stddev(0.1);
  pitch_filter_.set_proc_stddev(0.1);

  pub_measured_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("debug/measured_pose", 1);
}

/*
 * updatePredictFrequency
 */
void EKFLocalizer::updatePredictFrequency()
{
  if (last_predict_time_) {
    if (get_clock()->now() < *last_predict_time_) {
      RCLCPP_WARN(get_logger(), "Detected jump back in time");
    } else {
      ekf_rate_ = 1.0 / (get_clock()->now() - *last_predict_time_).seconds();
      DEBUG_INFO(get_logger(), "[EKF] update ekf_rate_ to %f hz", ekf_rate_);
      ekf_dt_ = 1.0 / std::max(ekf_rate_, 0.1);

      /* Update discrete proc_cov*/
      proc_cov_vx_d_ = std::pow(proc_stddev_vx_c_ * ekf_dt_, 2.0);
      proc_cov_wz_d_ = std::pow(proc_stddev_wz_c_ * ekf_dt_, 2.0);
      proc_cov_yaw_d_ = std::pow(proc_stddev_yaw_c_ * ekf_dt_, 2.0);
      proc_cov_yaw_bias_d_ = std::pow(proc_stddev_yaw_bias_c_ * ekf_dt_, 2.0);
    }
  }
  last_predict_time_ = std::make_shared<const rclcpp::Time>(get_clock()->now());
}

/*
 * timerCallback
 */
void EKFLocalizer::timerCallback()
{
  DEBUG_INFO(get_logger(), "========================= timer called =========================");

  /* update predict frequency with measured timer rate */
  updatePredictFrequency();

  /* predict model in EKF */
  DEBUG_INFO(get_logger(), "------------------------- start prediction -------------------------");
  predictKinematicsModel();
  DEBUG_INFO(get_logger(), "------------------------- end prediction -------------------------\n");

  /* pose measurement update */
  if (!current_pose_info_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Pose -------------------------");

    int pose_info_queue_size = static_cast<int>(current_pose_info_queue_.size());
    for (int i = 0; i < pose_info_queue_size; ++i) {
      PoseInfo pose_info = current_pose_info_queue_.front();
      current_pose_info_queue_.pop();
      measurementUpdatePose(*pose_info.pose);
      ++pose_info.counter;
      if (pose_info.counter < pose_smoothing_steps_) {
        current_pose_info_queue_.push(pose_info);
      }
    }
    DEBUG_INFO(get_logger(), "------------------------- end Pose -------------------------\n");
  }

  /* twist measurement update */
  if (!current_twist_info_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Twist -------------------------");

    int twist_info_queue_size = static_cast<int>(current_twist_info_queue_.size());
    for (int i = 0; i < twist_info_queue_size; ++i) {
      TwistInfo twist_info = current_twist_info_queue_.front();
      current_twist_info_queue_.pop();
      measurementUpdateTwist(*twist_info.twist);
      ++twist_info.counter;
      if (twist_info.counter < twist_smoothing_steps_) {
        current_twist_info_queue_.push(twist_info);
      }
    }
    DEBUG_INFO(get_logger(), "------------------------- end Twist -------------------------\n");
  }

  const Eigen::Vector3d translation(
      ekf_.getXelement(0),
      ekf_.getXelement(1),
      z_filter_.get_x()
  );
  const double roll = roll_filter_.get_x();
  const double pitch = pitch_filter_.get_x();
  const double unbiased_yaw = ekf_.getXelement(2);
  const double yaw_bias = ekf_.getXelement(3);
  const rclcpp::Time stamp = this->now();

  Eigen::Isometry3d ekf_pose;
  ekf_pose.translation() = translation;
  ekf_pose.linear() =
    rotationlib::RPYToQuaternionXYZ(roll, pitch, unbiased_yaw + yaw_bias).toRotationMatrix();

  current_ekf_pose_ = MakePoseStamped(ekf_pose, stamp, pose_frame_id_);

  Eigen::Isometry3d ekf_unbiased_pose;
  ekf_unbiased_pose.translation() = translation;
  ekf_unbiased_pose.linear() =
    rotationlib::RPYToQuaternionXYZ(roll, pitch, unbiased_yaw).toRotationMatrix();

  const auto current_ekf_pose_no_yawbias_ =
    MakePoseStamped(ekf_unbiased_pose, stamp, pose_frame_id_);

  const Eigen::Vector3d linear(ekf_.getXelement(4), 0, 0);
  const Eigen::Vector3d angular(0, 0, ekf_.getXelement(5));
  const auto current_ekf_twist_ = MakeTwistStamped(linear, angular, this->now(), "base_link");

  /* publish ekf result */
  publishEstimateResult(
    dim_x_, ekf_, this->now(),
    current_ekf_pose_, current_ekf_pose_no_yawbias_, current_ekf_twist_, current_pose_info_queue_,
    pub_pose_, pub_pose_no_yawbias_, pub_twist_cov_, pub_pose_cov_, pub_odom_,
    pub_pose_cov_no_yawbias_, pub_twist_, pub_measured_pose_);
}

/*
 * timerTFCallback
 */
void EKFLocalizer::timerTFCallback()
{
  if (current_ekf_pose_.header.frame_id == "") {
    return;
  }

  const Eigen::Isometry3d pose = GetIsometry3d(current_ekf_pose_.pose);
  const std::string frame_id = current_ekf_pose_.header.frame_id;
  const std::string child_frame_id = "base_link";
  const rclcpp::Time stamp = this->now();
  const auto msg = MakeTransformStamped(pose, stamp, frame_id, child_frame_id);
  tf_br_->sendTransform(msg);
}

/*
 * getTransformFromTF
 */
bool EKFLocalizer::getTransformFromTF(
  std::string parent_frame, std::string child_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  tf2::BufferCore tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  rclcpp::sleep_for(std::chrono::milliseconds(100));
  if (parent_frame.front() == '/') {
    parent_frame.erase(0, 1);
  }
  if (child_frame.front() == '/') {
    child_frame.erase(0, 1);
  }

  for (int i = 0; i < 50; ++i) {
    try {
      transform = tf_buffer.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);
      return true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }
  return false;
}

/*
 * callbackInitialPose
 */
void EKFLocalizer::callbackInitialPose(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
{
  geometry_msgs::msg::TransformStamped transform;
  if (!getTransformFromTF(pose_frame_id_, initialpose->header.frame_id, transform)) {
    RCLCPP_ERROR(
      get_logger(), "[EKF] TF transform failed. parent = %s, child = %s", pose_frame_id_.c_str(),
      initialpose->header.frame_id.c_str());
  }

  // TODO(mitsudome-r) need mutex

  const Eigen::Vector3d initial_position = ToVector3d(initialpose->pose.pose.position);
  const Eigen::Vector3d translation = ToVector3d(transform.transform.translation);
  const Eigen::Vector3d t = initial_position + translation;
  const double initial_yaw = tf2::getYaw(initialpose->pose.pose.orientation);
  const double yaw = tf2::getYaw(transform.transform.rotation);

  current_ekf_pose_.pose.position.z = t(2);

  const Vector6d X = (Vector6d() << t(0), t(1), initial_yaw + yaw, 0.0, 0.0, 0.0).finished();

  const Matrix6d covariance = GetEigenCovariance(initialpose->pose.covariance);
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x_, dim_x_);
  P(0, 0) = covariance(0, 0);
  P(1, 1) = covariance(1, 1);
  P(2, 2) = covariance(5, 5);
  P(3, 3) = 0.0001;
  P(4, 4) = 0.01;
  P(5, 5) = 0.01;

  ekf_.init(X, P, extend_state_step_);

  updateSimple1DFilters(*initialpose);

  while (!current_pose_info_queue_.empty()) current_pose_info_queue_.pop();
}

/*
 * callbackPoseWithCovariance
 */
void EKFLocalizer::callbackPoseWithCovariance(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  PoseInfo pose_info = {msg, 0};
  current_pose_info_queue_.push(pose_info);

  updateSimple1DFilters(*msg);
}

/*
 * callbackTwistWithCovariance
 */
void EKFLocalizer::callbackTwistWithCovariance(
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  TwistInfo twist_info = {msg, 0};
  current_twist_info_queue_.push(twist_info);
}

/*
 * initEKF
 */
void EKFLocalizer::initEKF()
{
  const Vector6d X = Vector6d::Zero(dim_x_, 1);
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(dim_x_, dim_x_) * 1.0E15;  // for x & y
  P(2, 2) = 50.0;                                            // for yaw
  P(3, 3) = proc_cov_yaw_bias_d_;                          // for yaw bias
  P(4, 4) = 1000.0;                                            // for vx
  P(5, 5) = 50.0;                                              // for wz

  ekf_.init(X, P, extend_state_step_);
}

/*
 * predictKinematicsModel
 */
void EKFLocalizer::predictKinematicsModel()
{
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

  const Eigen::MatrixXd X_curr = ekf_.getLatestX();  // current state
  DEBUG_PRINT_MAT(X_curr.transpose());

  const double unbiased_yaw = X_curr(2);
  const double yaw_bias = X_curr(3);
  const double vx = X_curr(4);
  const double wz = X_curr(5);
  const double dt = ekf_dt_;
  const double yaw = unbiased_yaw + yaw_bias;

  /* Update for latest state */
  Vector6d X_next;  // predicted state
  X_next(0) = X_curr(0) + vx * cos(yaw) * dt;  // dx = v * cos(yaw)
  X_next(1) = X_curr(1) + vx * sin(yaw) * dt;  // dy = v * sin(yaw)
  X_next(2) = X_curr(2) + (wz)*dt;                    // dyaw = omega + omega_bias
  X_next(3) = yaw_bias;
  X_next(4) = vx;
  X_next(5) = wz;

  X_next(2) = std::atan2(std::sin(X_next(2)), std::cos(X_next(2)));

  /* Set A matrix for latest state */
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dim_x_, dim_x_);
  A(0, 2) = -vx * sin(yaw) * dt;
  A(0, 3) = -vx * sin(yaw) * dt;
  A(0, 4) = cos(yaw) * dt;
  A(1, 2) = vx * cos(yaw) * dt;
  A(1, 3) = vx * cos(yaw) * dt;
  A(1, 4) = sin(yaw) * dt;
  A(2, 5) = dt;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x_, dim_x_);

  Q(0, 0) = 0.0;
  Q(1, 1) = 0.0;
  Q(2, 2) = proc_cov_yaw_d_;         // for yaw
  Q(3, 3) = proc_cov_yaw_bias_d_;  // for yaw bias
  Q(4, 4) = proc_cov_vx_d_;            // for vx
  Q(5, 5) = proc_cov_wz_d_;            // for wz

  ekf_.predictWithDelay(X_next, A, Q);

  // debug
  const Eigen::MatrixXd X_result = ekf_.getLatestX();
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdatePose
 */
void EKFLocalizer::measurementUpdatePose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  if (pose.header.frame_id != pose_frame_id_) {
    warning_.WarnThrottle(
      2000,
      fmt::format(
        "pose frame_id is {}, but pose_frame is set as {}. They must be same.",
        pose.header.frame_id, pose_frame_id_));
  }

  // current state
  const Eigen::MatrixXd X_curr = ekf_.getLatestX();
  DEBUG_PRINT_MAT(X_curr.transpose());

  const rclcpp::Time t_curr = this->now();

  /* Calculate delay step */
  double delay_time = (t_curr - pose.header.stamp).seconds() + pose_additional_delay_;
  if (delay_time < 0.0) {
    delay_time = 0.0;
    warning_.WarnThrottle(
      1000,
      fmt::format("Pose time stamp is inappropriate, set delay to 0[s]. delay = {}", delay_time));
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1) {
    warning_.WarnThrottle(
      1000,
      fmt::format(
        "Pose delay exceeds the compensation limit, ignored. delay: {}[s], limit = "
        "extend_state_step * ekf_dt : {} [s]",
        delay_time, extend_state_step_ * ekf_dt_));
    return;
  }
  DEBUG_INFO(get_logger(), "delay_time: %f [s]", delay_time);

  /* Set yaw */
  double yaw = tf2::getYaw(pose.pose.pose.orientation);
  const double ekf_yaw = ekf_.getXelement(delay_step * dim_x_ + 2);
  const double yaw_error = normalizeYaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi
  yaw = yaw_error + ekf_yaw;

  /* Set measurement matrix */
  const Eigen::Vector3d y(pose.pose.pose.position.x, pose.pose.pose.position.y, yaw);

  if (isnan(y.array()).any() || isinf(y.array()).any()) {
    RCLCPP_WARN(
      get_logger(),
      "[EKF] pose measurement matrix includes NaN of Inf. ignore update. check pose message.");
    return;
  }

  /* Gate */
  const Eigen::Vector3d y_ekf(
    ekf_.getXelement(delay_step * dim_x_ + 0),
    ekf_.getXelement(delay_step * dim_x_ + 1),
    ekf_yaw);

  constexpr int dim_y = 3;  // pos_x, pos_y, yaw, depending on Pose output
  const Eigen::MatrixXd P_y = ekf_.getLatestP().block(0, 0, dim_y, dim_y);
  if (!mahalanobisGate(pose_gate_dist_, y_ekf, y, P_y)) {
    warning_.WarnThrottle(
      2000,
      "[EKF] Pose measurement update, mahalanobis distance is over limit. ignore "
      "measurement data.");
    return;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, 0) = 1.0;    // for pos x
  C(1, 1) = 1.0;    // for pos y
  C(2, 2) = 1.0;  // for yaw

  /* Set measurement noise covariance */
  const Matrix6d covariance = GetEigenCovariance(pose.pose.covariance);
  Eigen::Matrix3d R;
  R <<
    covariance(0, 0), covariance(0, 1), covariance(0, 5),
    covariance(1, 0), covariance(1, 1), covariance(1, 5),
    covariance(5, 0), covariance(5, 1), covariance(5, 5);

  /* In order to avoid a large change at the time of updating,
   * measurement update is performed by dividing at every step. */
  R *= pose_smoothing_steps_;

  ekf_.updateWithDelay(y, C, R, delay_step);

  // debug
  const Eigen::MatrixXd X_result = ekf_.getLatestX();
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

/*
 * measurementUpdateTwist
 */
void EKFLocalizer::measurementUpdateTwist(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist)
{
  if (twist.header.frame_id != "base_link") {
    warning_.WarnThrottle(2000, "twist frame_id must be base_link");
  }

  // current state
  const Eigen::MatrixXd X_curr = ekf_.getLatestX();
  DEBUG_PRINT_MAT(X_curr.transpose());

  const rclcpp::Time t_curr = this->now();

  /* Calculate delay step */
  double delay_time = (t_curr - twist.header.stamp).seconds() + twist_additional_delay_;
  if (delay_time < 0.0) {
    warning_.WarnThrottle(
      1000,
      fmt::format(
        "Twist time stamp is inappropriate (delay = {} [s]), set delay to 0[s].",
        delay_time));
    delay_time = 0.0;
  }
  int delay_step = std::roundf(delay_time / ekf_dt_);
  if (delay_step > extend_state_step_ - 1) {
    warning_.WarnThrottle(
      1000,
      fmt::format(
        "Twist delay exceeds the compensation limit, ignored. delay: {}[s], limit = "
        "extend_state_step * ekf_dt : {} [s]", delay_time, extend_state_step_ * ekf_dt_));
    return;
  }
  DEBUG_INFO(get_logger(), "delay_time: %f [s]", delay_time);

  constexpr int dim_y = 2;  // vx, wz
  /* Set measurement matrix */
  const Eigen::Vector2d y(twist.twist.twist.linear.x, twist.twist.twist.angular.z);

  if (isnan(y.array()).any() || isinf(y.array()).any()) {
    RCLCPP_WARN(
      get_logger(),
      "[EKF] twist measurement matrix includes NaN of Inf. ignore update. check twist message.");
    return;
  }

  /* Gate */
  const Eigen::Vector2d y_ekf(
    ekf_.getXelement(delay_step * dim_x_ + 4),
    ekf_.getXelement(delay_step * dim_x_ + 5));
  const Eigen::MatrixXd P_y = ekf_.getLatestP().block(4, 4, dim_y, dim_y);
  if (!mahalanobisGate(twist_gate_dist_, y_ekf, y, P_y)) {
    warning_.WarnThrottle(
      2000,
      "[EKF] Twist measurement update, mahalanobis distance is over limit. ignore "
      "measurement data.");
    return;
  }

  DEBUG_PRINT_MAT(y.transpose());
  DEBUG_PRINT_MAT(y_ekf.transpose());
  DEBUG_PRINT_MAT((y - y_ekf).transpose());

  /* Set measurement matrix */
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dim_y, dim_x_);
  C(0, 4) = 1.0;  // for vx
  C(1, 5) = 1.0;  // for wz

  /* Set measurement noise covariance */
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_y, dim_y);
  const Matrix6d covariance = GetEigenCovariance(twist.twist.covariance);
  R(0, 0) = covariance(0, 0);   // vx - vx
  R(0, 1) = covariance(0, 5);   // vx - wz
  R(1, 0) = covariance(5, 0);   // wz - vx
  R(1, 1) = covariance(5, 5);   // wz - wz

  /* In order to avoid a large change by update, measurement update is performed
   * by dividing at every step. measurement update is performed by dividing at every step. */
  R *= twist_smoothing_steps_;

  ekf_.updateWithDelay(y, C, R, delay_step);

  // debug
  const Eigen::MatrixXd X_result = ekf_.getLatestX();
  DEBUG_PRINT_MAT(X_result.transpose());
  DEBUG_PRINT_MAT((X_result - X_curr).transpose());
}

void EKFLocalizer::updateSimple1DFilters(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
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
