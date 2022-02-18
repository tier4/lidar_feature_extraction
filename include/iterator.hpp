// BSD 3-Clause License
// Copyright (c) 2020, Tixiao Shan, Takeshi Ishita
// All rights reserved.

#ifndef ITERATOR_HPP_
#define ITERATOR_HPP_

template<typename Iterator>
using ElementType = typename std::iterator_traits<typename Iterator::iterator>::value_type;

#endif  // ITERATOR_HPP_
