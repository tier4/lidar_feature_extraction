#ifndef JACOBIAN_DRP_DQ_HPP_
#define JACOBIAN_DRP_DQ_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Matrix<double, 3, 4> DRpdq(const Eigen::Quaterniond & q, const Eigen::Vector3d & p);

#endif  // JACOBIAN_DRP_DQ_HPP_
