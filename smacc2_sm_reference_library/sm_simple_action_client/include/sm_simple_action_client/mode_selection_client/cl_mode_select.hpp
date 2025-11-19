// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
*

Author: Sukhraj Klair
******************************************************************************************************************/

#ifndef ROBOT_STATE_MACHINE__CLIENTS__CL_MODE_SELECT_HPP_
#define ROBOT_STATE_MACHINE__CLIENTS__CL_MODE_SELECT_HPP_

#include "example_interfaces/msg/int32.hpp"
#include "smacc2/client_bases/smacc_subscriber_client.hpp"

namespace robot_state_machine
{

using namespace smacc2::client_bases;

//declare the events
template <typename TSource, typename TOrthogonal>
struct EvAutonomousMode : sc::event<EvAutonomousMode<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvManualMode : sc::event<EvManualMode<TSource, TOrthogonal>>
{
};

class ClModeSelect : public SmaccSubscriberClient<example_interfaces::msg::Int32>
{
public:
  ClModeSelect() : SmaccSubscriberClient<example_interfaces::msg::Int32>("mode_command") {}

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    post_autonomous_mode_event_ = [this]()
    {
      RCLCPP_INFO(getLogger(), "ClModeSelect::post_autonomous_mode_event_");
      // auto event = new EvAutonomousMode<TSourceObject, TOrthogonal>();
      this->postEvent<EvAutonomousMode<TSourceObject, TOrthogonal>>();
    };
    post_manual_mode_event_ = [this]()
    {
      RCLCPP_INFO(getLogger(), "ClModeSelect::post_manual_mode_event_");
      // auto event = new EvManualMode<TSourceObject, TOrthogonal>();
      this->postEvent<EvManualMode<TSourceObject, TOrthogonal>>();
    };

    SmaccSubscriberClient<example_interfaces::msg::Int32>::onStateOrthogonalAllocation<
      TOrthogonal, TSourceObject>();
  }

  std::function<void()> post_manual_mode_event_;
  std::function<void()> post_autonomous_mode_event_;
};

}  // namespace robot_state_machine

#endif
