#include "drp_dq.hpp"

#include <gmock/gmock.h>

TEST(Jacobian, DRpdq)
{
  const Eigen::Vector3d p(1., 2., -1.);
  const Eigen::Quaterniond q0 = Eigen::Quaterniond(1., 1., -1., 1).normalized();
  const Eigen::Matrix<double, 3, 4> J = DRpdq(q0, p);

  const Eigen::Quaterniond dq = Eigen::Quaterniond(1., -0.1, 0.1, 0.01).normalized();
  const Eigen::Quaterniond q1 = q0 * dq;

  const Eigen::Vector4d wxyz(dq.w(), dq.x(), dq.y(), dq.z());

  EXPECT_TRUE((q0 * p + J * wxyz - q1 * p).norm() > 1e-6);
}
