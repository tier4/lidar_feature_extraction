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

#ifndef EKF_LOCALIZER__AGED_MESSAGE_QUEUE_HPP_
#define EKF_LOCALIZER__AGED_MESSAGE_QUEUE_HPP_

#include <queue>

template<typename Message>
class AgedMessageQueue
{
public:
  explicit AgedMessageQueue(const int max_age)
  : max_age_(max_age)
  {
  }

  size_t size() const
  {
    return msgs_.size();
  }

  void push(const Message & msg)
  {
    msgs_.push(msg);
    ages_.push(0);
  }

  Message pop()
  {
    const auto msg = msgs_.front();
    const int age = ages_.front() + 1;
    msgs_.pop();
    ages_.pop();

    if (age < max_age_) {
      msgs_.push(msg);
      ages_.push(age);
    }

    return msg;
  }

  void clear()
  {
    msgs_ = std::queue<Message>();
    ages_ = std::queue<int>();
  }

private:
  const int max_age_;
  std::queue<Message> msgs_;
  std::queue<int> ages_;
};

#endif  // EKF_LOCALIZER__AGED_MESSAGE_QUEUE_HPP_
