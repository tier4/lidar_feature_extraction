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

#include <pcl/common/transforms.h>

#include "lidar_feature_library/transform.hpp"

#include "lidar_feature_localization/optimizer.hpp"
#include "lidar_feature_localization/surface.hpp"


TEST(Surface, EstimatePlaneCoefficients)
{
  {
    const Eigen::MatrixXd X =
      (Eigen::MatrixXd(4, 2) <<
        2, 3,
        3, 4,
        4, 5,
        5, 6
      ).finished();

    const Eigen::Vector2d w = EstimatePlaneCoefficients(X);
    const Eigen::Vector2d expected(1, -1);
    EXPECT_THAT((w - expected).norm(), testing::Le(1e-6));
  }

  {
    const Eigen::MatrixXd X =
      (Eigen::MatrixXd(4, 3) <<
        1, 1, -1,
        2, 1, -2,
        2, -1, 0,
        4, -2, -1
      ).finished();
    const Eigen::Vector3d w = EstimatePlaneCoefficients(X);
    const Eigen::Vector3d expected(1, 1, 1);
    const Eigen::Vector4d bias = Eigen::Vector4d::Ones();
    EXPECT_THAT((X * w + bias).norm(), testing::Le(1e-6));
  }
}

TEST(Surface, CheckPointsDistributeAlongPlane)
{
  {
    const Eigen::MatrixXd X =
      (Eigen::MatrixXd(4, 2) <<
        2, 3,
        3, 4,
        4, 5,
        5, 6
      ).finished();
    const Eigen::Vector2d w(1, -1);
    EXPECT_TRUE(CheckPointsDistributeAlongPlane(X, w));
  }

  {
    const Eigen::MatrixXd X =
      (Eigen::MatrixXd(4, 2) <<
        2, 3,
        3, 4,
        5, 5,
        5, 6
      ).finished();
    const Eigen::Vector2d w(1, -1);
    EXPECT_FALSE(CheckPointsDistributeAlongPlane(X, w));
  }
}

TEST(Surface, ResidualApproximation)
{
  auto residual = [](
    const Eigen::Vector3d & w,
    const Eigen::Vector3d & theta,
    const Eigen::Vector3d & t,
    const Eigen::Vector3d & p) {
      const Eigen::Quaterniond q = AngleAxisToQuaternion(theta);

      Eigen::Isometry3d transform;
      transform.linear() = q.toRotationMatrix();
      transform.translation() = t;

      return SignedPointPlaneDistance(w, transform * p);
    };

  const Eigen::Vector3d w(1., 1., 1.);
  const Eigen::Vector3d theta0(0., 0., 0.);
  const Eigen::Vector3d t0(3, 2., 1.);

  const Eigen::Vector3d p(2., 4., 0.);

  const Eigen::Quaterniond q0 = AngleAxisToQuaternion(theta0);
  const Eigen::Matrix<double, 1, 7> drdqt = MakeJacobianRow(w, q0, p);
  const Eigen::Matrix<double, 7, 6> M = MakeM(q0);
  const Eigen::Matrix<double, 1, 6> J = drdqt * M;

  {
    const Eigen::Vector3d dtheta(0.1, 0.1, 0.1);
    const Eigen::Vector3d dt = Eigen::Vector3d::Zero();

    const Eigen::Vector3d theta1 = theta0 + dtheta;
    const Eigen::Vector3d t1 = t0 + dt;

    const double r0 = residual(w, theta0, t0, p);
    const double r1 = residual(w, theta1, t1, p);
    const Vector6d delta = (Vector6d() << dtheta, dt).finished();
    const double threshold = 0.1 * std::abs(r1 - r0);
    EXPECT_THAT(std::abs(r1 - (r0 + J * delta)), testing::Le(threshold));
  }

  {
    const Eigen::Vector3d dtheta = Eigen::Vector3d::Zero();
    const Eigen::Vector3d dt(0.1, 0.2, 0.4);

    const Eigen::Vector3d theta1 = theta0 + dtheta;
    const Eigen::Vector3d t1 = t0 + dt;

    const double r0 = residual(w, theta0, t0, p);
    const double r1 = residual(w, theta1, t1, p);
    const Vector6d delta = (Vector6d() << dtheta, dt).finished();
    const double threshold = 0.1 * std::abs(r1 - r0);
    EXPECT_THAT(std::abs(r1 - (r0 + J * delta)), testing::Le(threshold));
  }

  {
    const Eigen::Vector3d dtheta(0.1, -0.1, -0.1);
    const Eigen::Vector3d dt(0.1, 0.2, 0.4);

    const Eigen::Vector3d theta1 = theta0 + dtheta;
    const Eigen::Vector3d t1 = t0 + dt;

    const double r0 = residual(w, theta0, t0, p);
    const double r1 = residual(w, theta1, t1, p);
    const Vector6d delta = (Vector6d() << dtheta, dt).finished();
    const double threshold = 0.1 * std::abs(r1 - r0);
    EXPECT_THAT(std::abs(r1 - (r0 + J * delta)), testing::Le(threshold));
  }
}

Eigen::VectorXd Residuals(
  const std::vector<Eigen::Vector3d> & weights,
  const std::vector<Eigen::Vector3d> & points,
  const Eigen::Quaterniond & q,
  const Eigen::Vector3d & t)
{
  Eigen::Isometry3d transform;
  transform.linear() = q.toRotationMatrix();
  transform.translation() = t;

  Eigen::VectorXd r(weights.size());
  for (unsigned int i = 0; i < weights.size(); i++) {
    r(i) = SignedPointPlaneDistance(weights[i], transform * points[i]);
  };
  return r;
}

Eigen::MatrixXd DrDqt(
  const std::vector<Eigen::Vector3d> & weights,
  const std::vector<Eigen::Vector3d> & points,
  const Eigen::Quaterniond & q) {
  Eigen::MatrixXd J(weights.size(), 7);
  for (unsigned int i = 0; i < weights.size(); i++) {
    J.row(i) = MakeJacobianRow(weights[i], q, points[i]);
  }
  return J;
}

TEST(Surface, ErrorApproximation)
{
  const std::vector<Eigen::Vector3d> weights{
    Eigen::Vector3d(1., 1., 1.),
    Eigen::Vector3d(2., 1., 2.),
    Eigen::Vector3d(3., 1., -1.)
  };

  const std::vector<Eigen::Vector3d> points{
    Eigen::Vector3d(2., -1., -2.),
    Eigen::Vector3d(1., -1., -1.),
    Eigen::Vector3d(1., 4., 7.)
  };

  const Eigen::Vector3d t0 = Eigen::Vector3d::Zero();
  const Eigen::Vector3d theta0 = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond q0 = AngleAxisToQuaternion(theta0);

  const Eigen::MatrixXd J = DrDqt(weights, points, q0) * MakeM(q0);

  const Eigen::Vector3d dtheta(0.1, -0.1, -0.1);
  const Eigen::Vector3d dt(0.1, 0.2, 0.4);

  const Eigen::Quaterniond q1 = AngleAxisToQuaternion(theta0 + dtheta);
  const Eigen::Vector3d t1 = t0 + dt;

  const Eigen::VectorXd r0 = Residuals(weights, points, q0, t0);
  const Eigen::VectorXd r1 = Residuals(weights, points, q1, t1);

  const Vector6d delta = (Vector6d() << dtheta, dt).finished();

  const double threshold = 0.1 * (r1 - r0).squaredNorm();
  EXPECT_THAT((r1 - (r0 + J * delta)).squaredNorm(), testing::Le(threshold));
}

TEST(Surface, CalcUpdate)
{
  const std::vector<Eigen::Vector3d> weights{
    Eigen::Vector3d(1., 1., 1.),
    Eigen::Vector3d(2., 1., 2.),
    Eigen::Vector3d(3., 1., -1.),
    Eigen::Vector3d(4., 8., -3.)
  };

  const std::vector<Eigen::Vector3d> points{
    Eigen::Vector3d(1., -2., -4.),
    Eigen::Vector3d(2., -3., -3.),
    Eigen::Vector3d(2., 4., 1.),
    Eigen::Vector3d(1., 4., 2.)
  };

  const Eigen::Vector3d t0 = Eigen::Vector3d::Zero();
  const Eigen::Vector3d theta0 = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond q0 = AngleAxisToQuaternion(theta0);

  const Eigen::MatrixXd J = DrDqt(weights, points, q0) * MakeM(q0);
  const Eigen::VectorXd r0 = Residuals(weights, points, q0, t0);
  const Vector6d delta = CalcUpdate(J, r0);

  const Eigen::Quaterniond q1 = AngleAxisToQuaternion(theta0) * AngleAxisToQuaternion(delta.head(3));
  // const Eigen::Quaterniond q1 = AngleAxisToQuaternion(theta0 + delta.head(3));
  const Eigen::Vector3d t1 = t0 + delta.tail(3);

  const Eigen::VectorXd r1 = Residuals(weights, points, q1, t1);
  EXPECT_THAT(r1.squaredNorm(), testing::Le(r0.squaredNorm()));
}

template<typename T>
double SumSquaredDistance(
  const typename pcl::PointCloud<T>::Ptr & cloud0,
  const typename pcl::PointCloud<T>::Ptr & cloud1)
{
  assert(cloud0->size() == cloud1->size());
  double distance = 0;
  for (unsigned int i = 0; i < cloud0->size(); i++) {
    const T p0 = cloud0->at(i);
    const T p1 = cloud1->at(i);
    distance +=
      (p0.x - p1.x) * (p0.x - p1.x) +
      (p0.y - p1.y) * (p0.y - p1.y) +
      (p0.z - p1.z) * (p0.z - p1.z);
  }
  return distance;
}

TEST(Surface, Convergence)
{
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0., 0.01);

  auto normal = [&]() {
    return distribution(generator);
  };

  auto make_walls = [&](const int n) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr walls(new pcl::PointCloud<pcl::PointXYZ>());

    for (int y = 0; y < n; y++) {
      for (int x = 0; x < n; x++) {
        walls->push_back(pcl::PointXYZ(0.1 * x, 0.1 * y, normal()));
      }
    }

    for (int z = 0; z < n; z++) {
      for (int x = 0; x < n; x++) {
        walls->push_back(pcl::PointXYZ(0.1 * x, normal(), 0.1 * z));
      }
    }

    for (int y = 0; y < n; y++) {
      for (int z = 0; z < n; z++) {
        walls->push_back(pcl::PointXYZ(normal(), 0.1 * y, 0.1 * z));
      }
    }

    return walls;
  };

  const auto map = make_walls(20);

  const Eigen::Quaterniond q_ture = Eigen::Quaterniond(1.0, 0.1, 0.1, -0.1).normalized();
  const Eigen::Vector3d t_true(0.7, -0.3, 0.5);

  const Eigen::Isometry3d transform_true = MakeIsometry3d(q_ture, t_true);

  const auto scan = TransformPointCloud<pcl::PointXYZ>(transform_true.inverse(), map);

  const int n_neighbors = 5;
  const Surface surface(map, n_neighbors);

  const Eigen::Quaterniond q_initial = Eigen::Quaterniond::Identity();
  const Eigen::Vector3d t_initial = Eigen::Vector3d::Zero();
  const Eigen::Isometry3d initial_pose = MakeIsometry3d(q_initial, t_initial);

  const Optimizer<Surface, pcl::PointCloud<pcl::PointXYZ>::Ptr> optimizer(surface);
  const Eigen::Isometry3d transform_pred = optimizer.Run(scan, initial_pose);
  const auto transformed = TransformPointCloud<pcl::PointXYZ>(transform_pred, scan);

  const double threshold = SumSquaredDistance<pcl::PointXYZ>(map, scan);
  const double distance = SumSquaredDistance<pcl::PointXYZ>(map, transformed);

  EXPECT_THAT(distance, testing::Le(threshold));
  EXPECT_THAT(
    (transform_true.linear() - transform_pred.linear()).norm(),
    testing::Le(0.1));
  EXPECT_THAT(
    (transform_true.translation() - transform_pred.translation()).norm(),
    testing::Le(0.05));
}
