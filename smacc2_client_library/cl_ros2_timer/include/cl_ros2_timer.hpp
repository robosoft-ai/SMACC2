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

#include <boost/signals2.hpp>
#include <chrono>
#include <optional>
#include <smacc2/smacc.hpp>

namespace cl_ros2_timer
{
template <typename TSource, typename TOrthogonal>
struct EvTimer : sc::event<EvTimer<TSource, TOrthogonal>>
{
  /*
    ClRos2Timer *sender;
    rclcpp::TimerEvent timedata;

    EvTimer(ClRos2Timer *sender, const rclcpp::TimerEvent &timedata)
    {
        this->sender = sender;
        this->timedata = timedata;
    }
    */
};

class ClRos2Timer : public smacc2::ISmaccClient
{
public:
  ClRos2Timer(rclcpp::Duration duration, bool oneshot = false);

  virtual ~ClRos2Timer();

  virtual void onInitialize() override;

  template <typename T>
  boost::signals2::connection onTimerTick(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onTimerTick_, callback, object);
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->postTimerEvent_ = [this]() { this->postEvent<EvTimer<TSourceObject, TOrthogonal>>(); };
  }

protected:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Duration duration_;
  bool oneshot_;

  void timerCallback();
  std::function<void()> postTimerEvent_;
  smacc2::SmaccSignal<void()> onTimerTick_;
};
}  // namespace cl_ros2_timer
