// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef _REFERENCE_WRAPPER_LIDAR_ODOMETRY_H_
#define _REFERENCE_WRAPPER_LIDAR_ODOMETRY_H_

template<typename Element>
using ConstReferenceWrapper = std::reference_wrapper<const Element>;

template<typename Element>
using ConstReferenceVector = std::vector<ConstReferenceWrapper<Element>>;

#endif  /* _REFERENCE_WRAPPER_LIDAR_ODOMETRY_H_ */
