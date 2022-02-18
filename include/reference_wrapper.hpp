// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef REFERENCE_WRAPPER_HPP_
#define REFERENCE_WRAPPER_HPP_

#include <functional>
#include <vector>

template<typename Element>
using ConstReferenceWrapper = std::reference_wrapper<const Element>;

template<typename Element>
using ConstReferenceVector = std::vector<ConstReferenceWrapper<Element>>;

#endif  // REFERENCE_WRAPPER_HPP_
