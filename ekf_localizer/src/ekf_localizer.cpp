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

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <utility>

#include "lidar_feature_library/eigen.hpp"
#include "lidar_feature_library/ros_msg.hpp"
#include "rotationlib/quaternion.hpp"
#include "ekf_localizer/numeric.hpp"
#include "ekf_localizer/warning.hpp"

// clang-format off
#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
// clang-format on

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
  const geometry_msgs::msg::PoseStamped & current_unbiased_pose,
  const geometry_msgs::msg::PoseStamped & current_biased_pose,
  const geometry_msgs::msg::TwistStamped & current_twist,
  const std::queue<PoseInfo> & current_pose_info_queue_,
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub_pose_no_yawbias_,
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pub_odom_,
  const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr & pub_pose_cov_no_yawbias_,
  const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr & pub_measured_pose_)
{
  /* publish latest pose */
  pub_pose_no_yawbias_->publish(current_biased_pose);

  /* publish latest pose with covariance */
  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = current_time;
  pose_cov.header.frame_id = current_unbiased_pose.header.frame_id;
  pose_cov.pose.pose = current_unbiased_pose.pose;
  pose_cov.pose.covariance = EKFCovarianceToPoseMessageCovariance(P);

  geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_no_yawbias = pose_cov;
  pose_cov_no_yawbias.pose.pose = current_biased_pose.pose;
  pub_pose_cov_no_yawbias_->publish(pose_cov_no_yawbias);

  /* publish latest twist with covariance */
  geometry_msgs::msg::TwistWithCovarianceStamped twist_cov;
  twist_cov.header.stamp = current_time;
  twist_cov.header.frame_id = current_twist.header.frame_id;
  twist_cov.twist.twist = current_twist.twist;
  twist_cov.twist.covariance = EKFCovarianceToTwistMessageCovariance(P);

  /* publish latest odometry */
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = current_time;
  odometry.header.frame_id = current_unbiased_pose.header.frame_id;
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
}

double InitYawBias(const bool enable_yaw_bias_estimation, const double initial_value)
{
  if (enable_yaw_bias_estimation) {
    return initial_value;
  }
  return 0.;
}

std::chrono::nanoseconds DoubleToNanoSeconds(const double time) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(time));
}

TimeDelayKalmanFilter InitEKF(const int extend_state_step_, const double yaw_bias_variance)
{
  Matrix6d P = Matrix6d::Identity() * 1.0E15;    // for x & y
  P(2, 2) = 50.0;                                // for yaw
  P(3, 3) = yaw_bias_variance;                   // for yaw bias
  P(4, 4) = 1000.0;                              // for vx
  P(5, 5) = 50.0;                                // for wz

  TimeDelayKalmanFilter ekf;
  ekf.init(Vector6d::Zero(), P, extend_state_step_);
  return ekf;
}

EKFLocalizer::EKFLocalizer(const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options),
  warning_(this),
  pub_odom_(create_publisher<nav_msgs::msg::Odometry>("ekf_odom", 1)),
  pub_measured_pose_(create_publisher<geometry_msgs::msg::PoseStamped>("debug/measured_pose", 1)),
  pub_pose_no_yawbias_(
    create_publisher<geometry_msgs::msg::PoseStamped>("ekf_pose_without_yawbias", 1)),
  pub_pose_cov_no_yawbias_(create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance_without_yawbias", 1)),
  sub_initialpose_(create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1,
    std::bind(&EKFLocalizer::callbackInitialPose, this, std::placeholders::_1))),
  sub_pose_with_cov_(create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", 1,
    std::bind(&EKFLocalizer::callbackPoseWithCovariance, this, std::placeholders::_1))),
  sub_twist_with_cov_(create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance", 1,
    std::bind(&EKFLocalizer::callbackTwistWithCovariance, this, std::placeholders::_1))),
  last_predict_time_(std::nullopt),
  tf_br_(std::make_shared<tf2_ros::TransformBroadcaster>(
    std::shared_ptr<rclcpp::Node>(this, [](auto) {}))),
  default_frequency_(declare_parameter("predict_frequency", 50.0)),
  interval_(default_frequency_),
  ekf_dt_(ComputeInterval(default_frequency_)),
  show_debug_info_(declare_parameter("show_debug_info", false)),
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
  variances_(variance_.TimeScaledVariances(ekf_dt_)),
  ekf_(InitEKF(extend_state_step_, variances_(1)))
{

  /* convert to continuous to discrete */
  timer_control_ = rclcpp::create_timer(
    this, get_clock(), DoubleToNanoSeconds(ekf_dt_),
    std::bind(&EKFLocalizer::timerCallback, this));
  timer_tf_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(tf_rate_).period(),
    std::bind(&EKFLocalizer::timerTFCallback, this));

  z_filter_.set_proc_stddev(1.0);
  roll_filter_.set_proc_stddev(0.1);
  pitch_filter_.set_proc_stddev(0.1);
}

Vector6d PredictNextState(const Vector6d & x_curr, const double dt)
{
  const double biased_yaw = x_curr(2);
  const double yaw_bias = x_curr(3);
  const double vx = x_curr(4);
  const double wz = x_curr(5);
  const double yaw = biased_yaw + yaw_bias;

  Vector6d x_next;
  x_next <<
    x_curr(0) + vx * cos(yaw) * dt,  // dx = v * cos(yaw)
    x_curr(1) + vx * sin(yaw) * dt,  // dy = v * sin(yaw)
    normalizeYaw(x_curr(2) + wz*dt),                    // dyaw = omega + omega_bias
    yaw_bias,
    vx,
    wz;
  return x_next;
}

Matrix6d MatrixA(const Vector6d & x_curr, const double dt)
{
  const double biased_yaw = x_curr(2);
  const double yaw_bias = x_curr(3);
  const double vx = x_curr(4);
  const double yaw = biased_yaw + yaw_bias;

  /* Set A matrix for latest state */
  Matrix6d A = Matrix6d::Identity();
  A(0, 2) = -vx * sin(yaw) * dt;
  A(0, 3) = -vx * sin(yaw) * dt;
  A(0, 4) = cos(yaw) * dt;
  A(1, 2) = vx * cos(yaw) * dt;
  A(1, 3) = vx * cos(yaw) * dt;
  A(1, 4) = sin(yaw) * dt;
  A(2, 5) = dt;
  return A;
}

Matrix6d MatrixQ(
  const double yaw_covariance,
  const double yaw_bias_covariance,
  const double vx_covariance,
  const double wz_covariance)
{
  Vector6d q;
  q <<
    0.0,
    0.0,
    yaw_covariance,
    yaw_bias_covariance,
    vx_covariance,
    wz_covariance;
  return q.asDiagonal();
}

Eigen::Matrix<double, 3, 6> PoseObservationModel()
{
  /* Set measurement matrix */
  Eigen::Matrix<double, 3, 6> C = Eigen::Matrix<double, 3, 6>::Zero();
  C(0, 0) = 1.0;    // for pos x
  C(1, 1) = 1.0;    // for pos y
  C(2, 2) = 1.0;  // for yaw
  return C;
}

Eigen::Matrix<double, 2, 6> TwistObservationModel()
{
  Eigen::Matrix<double, 2, 6> C = Eigen::Matrix<double, 2, 6>::Zero();
  C(0, 4) = 1.0;  // for vx
  C(1, 5) = 1.0;  // for wz
  return C;
}

Eigen::Matrix3d PoseObservationCovariance(
  const Matrix6d & covariance,
  const double smoothing_steps)
{
  Eigen::Matrix3d R;
  R <<
    covariance(0, 0), covariance(0, 1), covariance(0, 5),
    covariance(1, 0), covariance(1, 1), covariance(1, 5),
    covariance(5, 0), covariance(5, 1), covariance(5, 5);
  /* In order to avoid a large change at the time of updating,
   * measurement update is performed by dividing at every step. */
  R *= smoothing_steps;
  return R;
}

Eigen::Matrix2d TwistObservationCovariance(
  const Matrix6d & covariance,
  const double smoothing_steps)
{
  Eigen::Matrix2d R;
  //   vx                wz
  R << covariance(0, 0), covariance(0, 5),   // vx
       covariance(5, 0), covariance(5, 5);   // wz

  /* In order to avoid a large change by update, measurement update is performed
   * by dividing at every step. measurement update is performed by dividing at every step. */
  R *= smoothing_steps;
  return R;
}

Eigen::Vector2d TwistMeasurementVector(const geometry_msgs::msg::Twist & twist)
{
  return Eigen::Vector2d(twist.linear.x, twist.angular.z);
}

Eigen::Vector2d TwistStateVector(const TimeDelayKalmanFilter & ekf, const int delay_step)
{
  return Eigen::Vector2d(
    ekf.getXelement(delay_step * DIM_X + 4),
    ekf.getXelement(delay_step * DIM_X + 5));
}

Eigen::Matrix2d TwistCovariance(const TimeDelayKalmanFilter & ekf)
{
  return ekf.getLatestP().block(4, 4, 2, 2);
}

double ComputeDelayTime(
  const rclcpp::Time & current_time,
  const rclcpp::Time & message_stamp,
  const double additional_delay)
{
  return (current_time - message_stamp).seconds() + additional_delay;
}

void CheckTwistFrameId(const Warning & warning_, const std::string & frame_id)
{
  if (frame_id != "base_link") {
    ShowFrameIdWarning(warning_, frame_id, "base_link");
  }
}

void CheckDelayTime(const Warning & warning_, const double delay_time)
{
  if (delay_time < 0.0) {
    ShowDelayTimeWarning(warning_, delay_time);
  }
}

int ComputeDelayStep(const double delay_time, const double dt)
{
  return std::roundf(std::max(delay_time, 0.) / dt);
}

void measurementUpdateTwist(
  TimeDelayKalmanFilter & ekf_,
  const rclcpp::Time & current_time,
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist,
  const Warning & warning_,
  const double ekf_dt_,
  const int extend_state_step_,
  const double twist_additional_delay_,
  const double twist_gate_dist_,
  const int twist_smoothing_steps_)
{
  CheckTwistFrameId(warning_, twist.header.frame_id);

  const double delay_time = ComputeDelayTime(
    current_time, twist.header.stamp, twist_additional_delay_);
  CheckDelayTime(warning_, delay_time);

  const int delay_step = ComputeDelayStep(delay_time, ekf_dt_);
  if (delay_step >= extend_state_step_) {
    ShowDelayStepWarning(warning_, delay_step, extend_state_step_);
    return;
  }

  const Eigen::Vector2d y = TwistMeasurementVector(twist.twist.twist);
  const Eigen::Vector2d y_ekf = TwistStateVector(ekf_, delay_step);
  const Eigen::Matrix2d P_y = TwistCovariance(ekf_);

  if (HasNan(y) || HasInf(y)) {
    ShowMeasurementMatrixNanInfWarning(warning_);
    return;
  }

  if (!mahalanobisGate(twist_gate_dist_, y_ekf, y, P_y)) {
    ShowMahalanobisGateWarning(warning_);
    return;
  }

  const Eigen::Matrix<double, 2, 6> C = TwistObservationModel();
  const Matrix6d covariance = GetEigenCovariance(twist.twist.covariance);
  const Eigen::Matrix2d R = TwistObservationCovariance(covariance, twist_smoothing_steps_);
  ekf_.updateWithDelay(y, C, R, delay_step);
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
  DEBUG_INFO(get_logger(), "========================= timer called =========================");

  /* update predict frequency with measured timer rate */
  const rclcpp::Time current_time = get_clock()->now();

  ekf_dt_ = interval_.Compute(current_time.seconds());
  variances_ = variance_.TimeScaledVariances(ekf_dt_);

  /* predict model in EKF */
  DEBUG_INFO(get_logger(), "------------------------- start prediction -------------------------");

  const Vector6d x_curr = ekf_.getLatestX();  // current state
  const Vector6d x_next = PredictNextState(x_curr, ekf_dt_);
  const Matrix6d A = MatrixA(x_curr, ekf_dt_);
  const Matrix6d Q = MatrixQ(variances_(0), variances_(1), variances_(2), variances_(3));

  ekf_.predictWithDelay(x_next, A, Q);
  DEBUG_INFO(get_logger(), "------------------------- end prediction -------------------------\n");

  /* pose measurement update */
  if (!current_pose_info_queue_.empty()) {
    DEBUG_INFO(get_logger(), "------------------------- start Pose -------------------------");

    for (size_t i = 0; i < current_pose_info_queue_.size(); ++i) {
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

    for (size_t i = 0; i < current_twist_info_queue_.size(); ++i) {
      TwistInfo twist_info = current_twist_info_queue_.front();
      current_twist_info_queue_.pop();
      measurementUpdateTwist(
        ekf_, this->now(), *twist_info.twist, warning_, ekf_dt_, extend_state_step_,
        twist_additional_delay_, twist_gate_dist_, twist_smoothing_steps_);
      ++twist_info.counter;
      if (twist_info.counter < twist_smoothing_steps_) {
        current_twist_info_queue_.push(twist_info);
      }
    }
    DEBUG_INFO(get_logger(), "------------------------- end Twist -------------------------\n");
  }

  const double x = ekf_.getXelement(0);
  const double y = ekf_.getXelement(1);
  const double z = z_filter_.get_x();
  const double roll = roll_filter_.get_x();
  const double pitch = pitch_filter_.get_x();
  const double biased_yaw = ekf_.getXelement(2);
  const double yaw_bias = ekf_.getXelement(3);
  const double yaw = biased_yaw + yaw_bias;
  const rclcpp::Time stamp = this->now();

  const Eigen::Isometry3d ekf_pose = MakePoseFromXYZRPY(x, y, z, roll, pitch, yaw);
  current_unbiased_pose_ = MakePoseStamped(ekf_pose, stamp, pose_frame_id_);

  const Eigen::Isometry3d ekf_biased_pose = MakePoseFromXYZRPY(x, y, z, roll, pitch, biased_yaw);

  const auto current_biased_pose = MakePoseStamped(ekf_biased_pose, stamp, pose_frame_id_);

  const Eigen::Vector3d linear(ekf_.getXelement(4), 0, 0);
  const Eigen::Vector3d angular(0, 0, ekf_.getXelement(5));
  const auto current_twist = MakeTwistStamped(linear, angular, this->now(), "base_link");

  /* publish ekf result */
  publishEstimateResult(
    ekf_.getLatestP(), this->now(),
    current_unbiased_pose_, current_biased_pose, current_twist, current_pose_info_queue_,
    pub_pose_no_yawbias_, pub_odom_, pub_pose_cov_no_yawbias_, pub_measured_pose_);
}

/*
 * timerTFCallback
 */
void EKFLocalizer::timerTFCallback()
{
  if (current_unbiased_pose_.header.frame_id == "") {
    return;
  }

  const Eigen::Isometry3d pose = GetIsometry3d(current_unbiased_pose_.pose);
  const std::string frame_id = current_unbiased_pose_.header.frame_id;
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
      warning_.Warn(ex.what());
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

  current_unbiased_pose_.pose.position.z = t(2);

  const Vector6d x = (Vector6d() << t(0), t(1), initial_yaw + yaw, 0.0, 0.0, 0.0).finished();

  const Matrix6d C = GetEigenCovariance(initialpose->pose.covariance);
  const Vector6d d = (Vector6d() << C(0, 0), C(1, 1), C(5, 5), 0.0001, 0.01, 0.01).finished();
  const Matrix6d P = d.asDiagonal();

  ekf_.init(x, P, extend_state_step_);

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

Eigen::Vector3d PoseMeasurementVector(
  const TimeDelayKalmanFilter & ekf,
  const geometry_msgs::msg::Pose & pose,
  const int delay_step)
{
  const double yaw = tf2::getYaw(pose.orientation);
  const double ekf_yaw = ekf.getXelement(delay_step * DIM_X + 2);
  const double yaw_error = normalizeYaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi

  return Eigen::Vector3d(pose.position.x, pose.position.y, yaw_error + ekf_yaw);
}

Eigen::Vector3d PoseStateVector(const TimeDelayKalmanFilter & ekf, const int delay_step)
{
  return Eigen::Vector3d(
    ekf.getXelement(delay_step * DIM_X + 0),
    ekf.getXelement(delay_step * DIM_X + 1),
    ekf.getXelement(delay_step * DIM_X + 2));
}

Eigen::Matrix3d PoseCovariance(const TimeDelayKalmanFilter & ekf)
{
  return ekf.getLatestP().block(0, 0, 3, 3);
}

/*
 * measurementUpdatePose
 */
void EKFLocalizer::measurementUpdatePose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  if (pose.header.frame_id != pose_frame_id_) {
    ShowFrameIdWarning(warning_, pose.header.frame_id, pose_frame_id_);
  }

  /* Calculate delay step */
  const double delay_time = ComputeDelayTime(
    this->now(), pose.header.stamp, pose_additional_delay_);
  CheckDelayTime(warning_, delay_time);

  const int delay_step = ComputeDelayStep(delay_time, ekf_dt_);
  if (delay_step >= extend_state_step_) {
    ShowDelayStepWarning(warning_, delay_step, extend_state_step_);
    return;
  }

  const Eigen::Vector3d y = PoseMeasurementVector(ekf_, pose.pose.pose, delay_step);
  const Eigen::Vector3d y_ekf = PoseStateVector(ekf_, delay_step);
  const Eigen::MatrixXd P_y = PoseCovariance(ekf_);

  if (HasNan(y) || HasInf(y)) {
    ShowMeasurementMatrixNanInfWarning(warning_);
    return;
  }

  if (!mahalanobisGate(pose_gate_dist_, y_ekf, y, P_y)) {
    ShowMahalanobisGateWarning(warning_);
    return;
  }

  const Eigen::Matrix<double, 3, 6> C = PoseObservationModel();
  const Matrix6d covariance = GetEigenCovariance(pose.pose.covariance);
  const Eigen::Matrix3d R = PoseObservationCovariance(covariance, pose_smoothing_steps_);

  ekf_.updateWithDelay(y, C, R, delay_step);
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
