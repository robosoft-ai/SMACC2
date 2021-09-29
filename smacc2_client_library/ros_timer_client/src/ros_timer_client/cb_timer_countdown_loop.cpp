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

#include <ros_timer_client/client_behaviors/cb_timer_countdown_loop.hpp>

namespace cl_ros_timer
{
CbTimerCountdownLoop::CbTimerCountdownLoop(unsigned long triggerTickCount)
: tickTriggerCount_(triggerTickCount), tickCounter_(0)
{
}

void CbTimerCountdownLoop::onClientTimerTickCallback()
{
  tickCounter_++;

  if (tickCounter_ % tickTriggerCount_ == 0)
  {
    onTimerTick_();
    postCountDownEvent_();
  }
}

void CbTimerCountdownLoop::onEntry()
{
  this->requiresClient(timerClient_);
  timerClient_->onTimerTick(&CbTimerCountdownLoop::onClientTimerTickCallback, this);
}

void CbTimerCountdownLoop::onExit() {}
}  // namespace cl_ros_timer
