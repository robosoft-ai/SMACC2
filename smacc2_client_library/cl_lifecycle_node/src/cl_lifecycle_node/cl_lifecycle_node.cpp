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

namespace
{
// Waits for the service, sends the request and waits for the response, checking `cancelled` and
// rclcpp::ok() every 100 ms; both waits are unbounded but abandonable. Returns nullptr if abandoned.
template <typename ServiceT>
typename ServiceT::Response::SharedPtr callService(
  const rclcpp::Logger & logger, typename rclcpp::Client<ServiceT>::SharedPtr client,
  typename ServiceT::Request::SharedPtr request, const ClLifecycleNode::CancelPredicate & cancelled,
  const std::string & what)
{
  const std::string serviceName = client->get_service_name();
  auto abandoned = [&cancelled]() { return (cancelled && cancelled()) || !rclcpp::ok(); };

  int laps = 0;
  while (!client->wait_for_service(std::chrono::milliseconds(100)))
  {
    if (abandoned())
    {
      RCLCPP_INFO(
        logger, "[ClLifecycleNode] %s abandoned while waiting for service %s", what.c_str(),
        serviceName.c_str());
      return nullptr;
    }
    if (++laps % 50 == 0)
    {
      RCLCPP_WARN(
        logger, "[ClLifecycleNode] still waiting for service %s (%d s)", serviceName.c_str(),
        laps / 10);
    }
  }

  auto future = client->async_send_request(request);
  laps = 0;
  while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
  {
    if (abandoned())
    {
      client->remove_pending_request(future);
      RCLCPP_INFO(
        logger, "[ClLifecycleNode] %s abandoned while waiting for the response of %s", what.c_str(),
        serviceName.c_str());
      return nullptr;
    }
    if (++laps % 50 == 0)
    {
      RCLCPP_WARN(
        logger, "[ClLifecycleNode] no response from %s after %d s", serviceName.c_str(), laps / 10);
    }
  }
  return future.get();
}
}  // namespace

bool ClLifecycleNode::changeState(uint8_t transition, const CancelPredicate & cancelled)
{
  auto logger = this->getNode()->get_logger();
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition;
  auto response = callService<lifecycle_msgs::srv::ChangeState>(
    logger, client_change_state_, request, cancelled, "transition " + std::to_string(transition));
  if (!response)
  {
    return false;
  }

  const std::string serviceName = client_change_state_->get_service_name();
  if (response->success)
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
  return response->success;
}

bool ClLifecycleNode::getState(uint8_t & state, const CancelPredicate & cancelled)
{
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto response = callService<lifecycle_msgs::srv::GetState>(
    this->getNode()->get_logger(), client_get_state_, request, cancelled, "get_state");
  if (!response)
  {
    return false;
  }
  state = response->current_state.id;
  return true;
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
  // Each primary state has its own shutdown transition id, so ask the node which state it is in.
  using lifecycle_msgs::msg::State;
  using lifecycle_msgs::msg::Transition;
  uint8_t state;
  if (!getState(state, cancelled))
  {
    return false;
  }
  switch (state)
  {
    case State::PRIMARY_STATE_UNCONFIGURED:
      return changeState(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN, cancelled);
    case State::PRIMARY_STATE_INACTIVE:
      return changeState(Transition::TRANSITION_INACTIVE_SHUTDOWN, cancelled);
    case State::PRIMARY_STATE_ACTIVE:
      return changeState(Transition::TRANSITION_ACTIVE_SHUTDOWN, cancelled);
    default:
      RCLCPP_ERROR(
        this->getNode()->get_logger(),
        "[ClLifecycleNode] %s is in lifecycle state %d, which has no shutdown transition",
        nodeName_.c_str(), state);
      return false;
  }
}

}  // namespace cl_lifecycle_node
