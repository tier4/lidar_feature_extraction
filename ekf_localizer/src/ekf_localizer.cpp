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
  const Eigen::Isometry3d & biased_pose,
  const Eigen::Vector3d & linear,
  const Eigen::Vector3d & angular,
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pub_odom_,
  const rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr & pub_biased_pose_)
{
  geometry_msgs::msg::PoseWithCovarianceStamped biased_pose_msg;
  biased_pose_msg.pose.pose = MakePose(biased_pose);
  biased_pose_msg.pose.covariance = EKFCovarianceToPoseMessageCovariance(P);
  biased_pose_msg.header.stamp = current_time;
  biased_pose_msg.header.frame_id = pose_frame_id;
  pub_biased_pose_->publish(biased_pose_msg);

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
  pub_biased_pose_(create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_biased_pose_with_covariance", 1)),
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

Matrix6d StateTransitionModel(const Vector6d & x_curr, const double dt)
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

Matrix6d ProcessNoiseCovariance(const Eigen::Vector4d & variances)
{
  Vector6d q;
  q << 0., 0., variances(0), variances(1), variances(2), variances(3);
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

Eigen::Vector3d PoseMeasurementVector(
  const TimeDelayKalmanFilter & ekf,
  const geometry_msgs::msg::Pose & pose,
  const int delay_step)
{
  const double yaw = tf2::getYaw(pose.orientation);
  const double ekf_yaw = ekf.getXelement(delay_step, 2);
  const double yaw_error = normalizeYaw(yaw - ekf_yaw);  // normalize the error not to exceed 2 pi

  return Eigen::Vector3d(pose.position.x, pose.position.y, yaw_error + ekf_yaw);
}

Eigen::Vector3d PoseStateVector(const TimeDelayKalmanFilter & ekf, const int delay_step)
{
  return Eigen::Vector3d(
    ekf.getXelement(delay_step, 0),
    ekf.getXelement(delay_step, 1),
    ekf.getXelement(delay_step, 2));
}

Eigen::Matrix3d PoseCovariance(const TimeDelayKalmanFilter & ekf)
{
  return ekf.getLatestP().block(0, 0, 3, 3);
}

Eigen::Vector2d TwistMeasurementVector(const geometry_msgs::msg::Twist & twist)
{
  return Eigen::Vector2d(twist.linear.x, twist.angular.z);
}

Eigen::Vector2d TwistStateVector(const TimeDelayKalmanFilter & ekf, const int delay_step)
{
  return Eigen::Vector2d(
    ekf.getXelement(delay_step, 4),
    ekf.getXelement(delay_step, 5));
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

bool CheckFrameId(
  const Warning & warning_,
  const std::string & frame_id,
  const std::string & expected_frame_id)
{
  const bool good = frame_id == expected_frame_id;
  if (!good) {
    ShowFrameIdWarning(warning_, frame_id, expected_frame_id);
  }
  return good;
}

bool CheckDelayTime(const Warning & warning_, const double delay_time)
{
  const bool good = delay_time >= 0.0;
  if (!good) {
    ShowDelayTimeWarning(warning_, delay_time);
  }
  return good;
}

bool CheckDelayStep(const Warning & warning_, const int delay_step, const int max_delay_step)
{
  const bool good = delay_step < max_delay_step;
  if (!good) {
    ShowDelayStepWarning(warning_, delay_step, max_delay_step);
  }
  return good;
}

bool CheckMeasurementMatrixNanInf(const Warning & warning_, const Eigen::MatrixXd & M) {
  const bool good = !HasNan(M) && !HasInf(M);

  if (!good) {
    ShowMeasurementMatrixNanInfWarning(warning_);
  }
  return good;
}

bool CheckMahalanobisGate(
  const Warning & warning_,
  const double & dist_max,
  const Eigen::MatrixXd & x1,
  const Eigen::MatrixXd & x2,
  const Eigen::MatrixXd & cov)
{
  const bool good = mahalanobisGate(dist_max, x1, x2, cov);
  if (!good) {
    ShowMahalanobisGateWarning(warning_);
  }
  return good;
}

int ComputeDelayStep(const double delay_time, const double dt)
{
  return std::roundf(std::max(delay_time, 0.) / dt);
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
  /* update predict frequency with measured timer rate */
  const rclcpp::Time current_time = get_clock()->now();

  const double dt = interval_.Compute(current_time.seconds());

  const Vector6d x_curr = ekf_.getLatestX();  // current state
  const Vector6d x_next = PredictNextState(x_curr, dt);
  const Matrix6d A = StateTransitionModel(x_curr, dt);
  const Matrix6d Q = ProcessNoiseCovariance(variance_.TimeScaledVariances(dt));

  ekf_.predictWithDelay(x_next, A, Q);

  /* pose measurement update */
  const size_t n_pose_msgs = pose_messages_.size();
  for (size_t i = 0; i < n_pose_msgs; ++i) {
    const auto pose = pose_messages_.pop();

    CheckFrameId(warning_, pose->header.frame_id, pose_frame_id_);

    const double delay_time = ComputeDelayTime(
      this->now(), pose->header.stamp, pose_additional_delay_);
    CheckDelayTime(warning_, delay_time);

    const int delay_step = ComputeDelayStep(delay_time, dt);
    if (!CheckDelayStep(warning_, delay_step, extend_state_step_)) {
      continue;
    }

    const Eigen::Vector3d y = PoseMeasurementVector(ekf_, pose->pose.pose, delay_step);
    const Eigen::Vector3d y_ekf = PoseStateVector(ekf_, delay_step);
    const Eigen::Matrix3d P_y = PoseCovariance(ekf_);

    if (!CheckMeasurementMatrixNanInf(warning_, y)) {
      continue;
    }

    if (!CheckMahalanobisGate(warning_, pose_gate_dist_, y_ekf, y, P_y)) {
      continue;
    }

    const Eigen::Matrix<double, 3, 6> C = PoseObservationModel();
    const Matrix6d covariance = GetEigenCovariance(pose->pose.covariance);
    const Eigen::Matrix3d R = PoseObservationCovariance(covariance, pose_smoothing_steps_);

    ekf_.updateWithDelay(y, C, R, delay_step);
  }

  /* twist measurement update */
  const size_t n_twist_msgs = twist_messages_.size();
  for (size_t i = 0; i < n_twist_msgs; ++i) {
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
    const Eigen::Vector2d y_ekf = TwistStateVector(ekf_, delay_step);
    const Eigen::Matrix2d P_y = TwistCovariance(ekf_);

    if (!CheckMeasurementMatrixNanInf(warning_, y)) {
      continue;
    }

    if (!CheckMahalanobisGate(warning_, twist_gate_dist_, y_ekf, y, P_y)) {
      continue;
    }

    const Eigen::Matrix<double, 2, 6> C = TwistObservationModel();
    const Matrix6d covariance = GetEigenCovariance(twist->twist.covariance);
    const Eigen::Matrix2d R = TwistObservationCovariance(covariance, twist_smoothing_steps_);
    ekf_.updateWithDelay(y, C, R, delay_step);
  }

  const Vector6d x_est = ekf_.getLatestX();
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
  const Eigen::Isometry3d biased_pose = MakePoseFromXYZRPY(x, y, z, roll, pitch, biased_yaw);

  const Eigen::Vector3d linear(vx, 0, 0);
  const Eigen::Vector3d angular(0, 0, wz);

  tf_br_->sendTransform(MakeTransformStamped(unbiased_pose, this->now(), pose_frame_id_, "base_link"));

  /* publish ekf result */
  publishEstimateResult(
    ekf_.getLatestP(), this->now(), pose_frame_id_,
    unbiased_pose, biased_pose, linear, angular, pub_odom_, pub_biased_pose_);
}

std::string EraseBeginSlash(const std::string & s)
{
  std::string a = s;
  if (a.front() == '/') {
    a.erase(0, 1);
  }
  return a;
}

/*
 * getTransformFromTF
 */
bool getTransformFromTF(
  const Warning & warning_,
  const std::string & parent_frame,
  const std::string & child_frame,
  geometry_msgs::msg::TransformStamped & transform)
{
  tf2::BufferCore tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  const std::string parent = EraseBeginSlash(parent_frame);
  const std::string child = EraseBeginSlash(child_frame);

  for (int i = 0; i < 50; ++i) {
    try {
      transform = tf_buffer.lookupTransform(parent, child, tf2::TimePointZero);
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
  if (!getTransformFromTF(warning_, pose_frame_id_, initialpose->header.frame_id, transform)) {
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

  const Vector6d x = (Vector6d() << t(0), t(1), initial_yaw + yaw, 0.0, 0.0, 0.0).finished();

  const Matrix6d C = GetEigenCovariance(initialpose->pose.covariance);
  const Vector6d d = (Vector6d() << C(0, 0), C(1, 1), C(5, 5), 0.0001, 0.01, 0.01).finished();
  const Matrix6d P = d.asDiagonal();

  ekf_.init(x, P, extend_state_step_);

  updateSimple1DFilters(*initialpose);

  pose_messages_.clear();
}

/*
 * callbackPoseWithCovariance
 */
void EKFLocalizer::callbackPoseWithCovariance(
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  pose_messages_.push(msg);

  updateSimple1DFilters(*msg);
}

/*
 * callbackTwistWithCovariance
 */
void EKFLocalizer::callbackTwistWithCovariance(
  geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  twist_messages_.push(msg);
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
