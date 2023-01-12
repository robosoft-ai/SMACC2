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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <smacc2/smacc.hpp>

namespace cl_lifecyclenode
{
struct EvLifecycleNodeChangeState : sc::event<EvLifecycleNodeChangeState>
{
  uint8_t state;
};

struct EvLifecycleOnUnconfigured : sc::event<EvLifecycleOnUnconfigured>
{
};

struct EvLifecycleOnErrorProcessing : sc::event<EvLifecycleOnErrorProcessing>
{
};

struct EvLifecycleOnCleaningUp : sc::event<EvLifecycleOnCleaningUp>
{
};

struct EvLifecycleOnConfiguring : sc::event<EvLifecycleOnConfiguring>
{
};

struct EvLifecycleOnInactive : sc::event<EvLifecycleOnInactive>
{
};

struct EvLifecycleOnShuttingDown : sc::event<EvLifecycleOnShuttingDown>
{
};

struct EvLifecycleOnShutdown : sc::event<EvLifecycleOnShutdown>
{
};

struct EvLifecycleOnFinalized : sc::event<EvLifecycleOnFinalized>
{
};

struct EvLifecycleOnDeactivating : sc::event<EvLifecycleOnDeactivating>
{
};

struct EvLifecycleOnActivating : sc::event<EvLifecycleOnActivating>
{
};

struct EvLifecycleOnActive : sc::event<EvLifecycleOnActive>
{
};

class ClLifecycleNode : public smacc2::ISmaccClient
{
public:
  ClLifecycleNode(std::string nodeName);

  virtual ~ClLifecycleNode();

  void onInitialize() override;

  void changeState(uint8_t state);

  virtual void onTransitionEvent(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);

private:
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_get_state_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_change_state_;

  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr
    subscription_transition_event_;

  std::string nodeName_;
  const std::string node_get_state_topic = "/get_state";
  const std::string node_change_state_topic = "/change_state";
  const std::string node_transition_event_topic = "/transition_event";
};

}  // namespace cl_lifecyclenode
