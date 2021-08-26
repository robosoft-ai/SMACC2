// Copyright 2021 RobosoftAI Inc.
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

#include <ros_timer_client/cl_ros_timer.hpp>

namespace cl_ros_timer
{
ClRosTimer::ClRosTimer(rclcpp::Duration duration, bool oneshot)
: duration_(duration), oneshot_(oneshot)
{
}

ClRosTimer::~ClRosTimer() { timer_->cancel(); }

void ClRosTimer::onInitialize()
{
  auto clock = this->getNode()->get_clock();

  timer_ = rclcpp::create_timer(
    this->getNode(), clock, duration_, std::bind(&ClRosTimer::timerCallback, this));
}

void ClRosTimer::timerCallback()
{
  if (!onTimerTick_.empty())
  {
    this->onTimerTick_();
  }
  postTimerEvent_();

  if (oneshot_)
  {
    this->timer_->cancel();
  }
}

}  // namespace cl_ros_timer
