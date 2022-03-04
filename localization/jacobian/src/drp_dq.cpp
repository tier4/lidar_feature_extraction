#include "drp_dq.hpp"

extern "C" {
#include "drp_dq.h"
}

#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Matrix<double, 3, 4> DRpdq(const Eigen::Quaterniond & q, const Eigen::Vector3d & p)
{
  Eigen::Matrix<double, 3, 4, Eigen::RowMajor> J;
  drp_dq(p(0), p(1), p(2), q.w(), q.x(), q.y(), q.z(), J.data());
  return Eigen::Matrix<double, 3, 4>(J);
}
