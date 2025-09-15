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

#include <smacc2/client_core_components/cp_ros2_timer.hpp>

namespace smacc2
{
namespace client_core_components
{

CpRos2Timer::CpRos2Timer(rclcpp::Duration duration, bool oneshot)
: duration_(duration), oneshot_(oneshot), initialized_(false)
{
}

CpRos2Timer::~CpRos2Timer()
{
  if (timer_)
  {
    timer_->cancel();
  }
}

void CpRos2Timer::onInitialize()
{
  if (!initialized_)
  {
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << this->getName() << "] Initializing ROS2 Timer with duration: "
                       << duration_.seconds() << "s, oneshot: " << (oneshot_ ? "true" : "false"));

    auto clock = this->getNode()->get_clock();
    timer_ = rclcpp::create_timer(
      this->getNode(), clock, duration_, std::bind(&CpRos2Timer::timerCallback, this));

    this->initialized_ = true;
  }
}

void CpRos2Timer::timerCallback()
{
  RCLCPP_DEBUG_STREAM(getLogger(), "[" << this->getName() << "] Timer tick");

  if (!onTimerTick_.empty())
  {
    onTimerTick_();
  }

  if (oneshot_)
  {
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << this->getName() << "] Oneshot timer completed, cancelling");
    timer_->cancel();
  }
}

void CpRos2Timer::startTimer()
{
  if (timer_ && timer_->is_canceled())
  {
    RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] Starting timer");
    timer_->reset();
  }
}

void CpRos2Timer::stopTimer()
{
  if (timer_)
  {
    RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] Stopping timer");
    timer_->cancel();
  }
}

void CpRos2Timer::cancelTimer()
{
  if (timer_)
  {
    RCLCPP_INFO_STREAM(getLogger(), "[" << this->getName() << "] Cancelling timer");
    timer_->cancel();
  }
}

}  // namespace client_core_components
}  // namespace smacc2
