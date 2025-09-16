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

#pragma once

#include <cl_ros2_timer/cl_ros2_timer.hpp>
#include <smacc2/smacc.hpp>

namespace cl_ros2_timer
{
class CbTimer : public smacc2::SmaccClientBehavior
{
public:
  void onEntry() override
  {
    this->requiresClient(timerClient_);

    // Get the timer listener component
    cl_ros2_timer::components::CpTimerListener1 * timerListener;
    this->requiresComponent(timerListener);

    // Connect to the timer listener component
    timerListener->onTimerCompleted(&CbTimer::onClientTimerTickCallback, this);
  }

  void onExit() override {}

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    this->postTimerEvent_ = [this]()
    { this->template postEvent<EvTimer<TSourceObject, TOrthogonal>>(); };
  }

  void onClientTimerTickCallback() { this->postTimerEvent_(); }

private:
  ClRos2Timer * timerClient_;
  std::function<void()> postTimerEvent_;
  boost::signals2::scoped_connection c_;
};
}  // namespace cl_ros2_timer
