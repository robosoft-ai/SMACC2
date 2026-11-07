// Copyright 2025 Robosoft Inc.
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
#include <chrono>
#include <cl_lifecycle_node/cl_lifecycle_node.hpp>
#include <cl_lifecycle_node/components/cp_lifecycle_event_monitor.hpp>
#include <future>
#include <string>

namespace cl_lifecycle_node
{
ClLifecycleNode::ClLifecycleNode(std::string node_name)
: nodeName_(node_name), eventMonitor_(nullptr)
{
}

ClLifecycleNode::~ClLifecycleNode() {}

void ClLifecycleNode::onInitialize()
{
  // Create service clients
  client_get_state_ = this->getNode()->create_client<lifecycle_msgs::srv::GetState>(
    this->nodeName_ + node_get_state_topic);
  client_change_state_ = this->getNode()->create_client<lifecycle_msgs::srv::ChangeState>(
    this->nodeName_ + node_change_state_topic);
}

bool ClLifecycleNode::changeState(uint8_t transition, const CancelPredicate & cancelled)
{
  auto logger = this->getNode()->get_logger();
  const std::string serviceName = client_change_state_->get_service_name();
  auto abandoned = [&cancelled]() { return (cancelled && cancelled()) || !rclcpp::ok(); };

  // The client is created in onInitialize() and the first request can follow within
  // milliseconds, before the service has been discovered. A request sent before discovery has
  // no reader and its future never completes, so wait for the server first. The wait is
  // unbounded but abandonable: the predicate is checked every 100 ms.
  int laps = 0;
  while (!client_change_state_->wait_for_service(std::chrono::milliseconds(100)))
  {
    if (abandoned())
    {
      RCLCPP_INFO(
        logger, "[ClLifecycleNode] transition %d abandoned while waiting for service %s",
        transition, serviceName.c_str());
      return false;
    }
    if (++laps % 50 == 0)
    {
      RCLCPP_WARN(
        logger, "[ClLifecycleNode] still waiting for service %s (%d s)", serviceName.c_str(),
        laps / 10);
    }
  }

  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto future = client_change_state_->async_send_request(request);
  laps = 0;
  while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    if (abandoned())
    {
      client_change_state_->remove_pending_request(future);
      RCLCPP_INFO(
        logger, "[ClLifecycleNode] transition %d abandoned while waiting for the response of %s",
        transition, serviceName.c_str());
      return false;
    }
    if (++laps % 50 == 0)
    {
      RCLCPP_WARN(
        logger, "[ClLifecycleNode] no response from %s after %d s", serviceName.c_str(), laps / 10);
    }
  }

  const bool accepted = future.get()->success;
  if (accepted)
  {
    RCLCPP_INFO(
      logger, "[ClLifecycleNode] transition %d accepted by %s", transition, serviceName.c_str());
  }
  else
  {
    RCLCPP_ERROR(
      logger, "[ClLifecycleNode] transition %d rejected by %s; no transition event will follow",
      transition, serviceName.c_str());
  }
  return accepted;
}

bool ClLifecycleNode::activate(const CancelPredicate & cancelled)
{
  return changeState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, cancelled);
}

bool ClLifecycleNode::deactivate(const CancelPredicate & cancelled)
{
  return changeState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, cancelled);
}

bool ClLifecycleNode::configure(const CancelPredicate & cancelled)
{
  return changeState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, cancelled);
}

bool ClLifecycleNode::cleanup(const CancelPredicate & cancelled)
{
  return changeState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, cancelled);
}

bool ClLifecycleNode::shutdown(const CancelPredicate & cancelled)
{
  return changeState(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, cancelled);
}

}  // namespace cl_lifecycle_node
