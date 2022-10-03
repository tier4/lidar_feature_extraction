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

#ifndef LIDAR_FEATURE_LOCALIZATION__STAMP_SORTED_OBJECTS_HPP_
#define LIDAR_FEATURE_LOCALIZATION__STAMP_SORTED_OBJECTS_HPP_

#include <map>
#include <mutex>
#include <tuple>


template<typename Object>
class StampSortedObjects
{
public:
  StampSortedObjects()
  {
  }

  void Insert(const double timestamp, const Object & object)
  {
    std::lock_guard<std::mutex> guard(mutex_);

    objects_[timestamp] = object;
  }

  std::tuple<double, Object> GetClosest(const double timestamp)
  {
    std::lock_guard<std::mutex> guard(mutex_);

    // g1 is the first element in map that satisfies g1 >= timestamp
    const auto g1 = objects_.lower_bound(timestamp);

    if (g1 == objects_.end()) {
      const auto last = std::prev(objects_.end());
      return std::make_tuple(last->first, last->second);
    }

    if (g1 == objects_.begin()) {
      const auto first = objects_.begin();
      return std::make_tuple(first->first, first->second);
    }

    const auto g0 = std::prev(g1);
    const auto [time0, object0] = *g0;
    const auto [time1, object1] = *g1;

    const double d0 = timestamp - time0;
    const double d1 = time1 - timestamp;
    return d0 < d1 ?
           std::make_tuple(time0, object0) :
           std::make_tuple(time1, object1);
  }

  size_t Size()
  {
    std::lock_guard<std::mutex> guard(mutex_);
    return objects_.size();
  }

  void RemoveOlderThan(const double timestamp)
  {
    std::lock_guard<std::mutex> guard(mutex_);

    // g is the first element in map that satisfies timestamp < g
    const auto g = objects_.upper_bound(timestamp);

    if (g == objects_.end()) {
      return;
    }

    const auto boundary = std::prev(g);

    auto curr = std::prev(objects_.end());
    while (curr != boundary) {
      auto prev = std::prev(curr);
      objects_.erase(curr);
      curr = prev;
    }
  }

private:
  std::mutex mutex_;
  std::map<double, Object> objects_;
};

#endif  // LIDAR_FEATURE_LOCALIZATION__STAMP_SORTED_OBJECTS_HPP_
