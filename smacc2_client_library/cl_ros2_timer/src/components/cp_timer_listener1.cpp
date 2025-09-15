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

#include <components/cp_timer_listener1.hpp>

namespace cl_ros2_timer
{
namespace components
{

CpTimerListener1::CpTimerListener1() {}

CpTimerListener1::~CpTimerListener1() {}

void CpTimerListener1::onInitialize()
{
  RCLCPP_INFO(getLogger(), "CpTimerListener1 initialization");

  // Require the CpRos2Timer component (similar to CpKeyboardListener1 requiring CpTopicSubscriber)
  this->requiresComponent(timerComponent_);

  // Connect to the timer component's tick signal
  timerComponent_->onTimerTick(&CpTimerListener1::onTimerTickCallback, this);
}

void CpTimerListener1::onTimerTickCallback()
{
  RCLCPP_DEBUG(getLogger(), "CpTimerListener1 received timer tick");

  // Emit our own signal for client behaviors to connect to
  if (!onTimerCompleted_.empty())
  {
    onTimerCompleted_();
  }

  // Post SMACC2 event for state transitions
  if (postTimerEvent_)
  {
    postTimerEvent_();
  }
}

}  // namespace components
}  // namespace cl_ros2_timer
