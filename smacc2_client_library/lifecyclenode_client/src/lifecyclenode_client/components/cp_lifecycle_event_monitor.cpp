// Copyright 2024 RobosoftAI Inc.
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

#include <lifecyclenode_client/components/cp_lifecycle_event_monitor.hpp>

namespace cl_lifecyclenode
{
CpLifecycleEventMonitor::CpLifecycleEventMonitor(std::string nodeName) : nodeName_(nodeName) {}

void CpLifecycleEventMonitor::onInitialize()
{
  // Phase 3: Create subscription to lifecycle transition events
  const std::string node_transition_event_topic = "/transition_event";

  subscription_ = getNode()->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
    nodeName_ + node_transition_event_topic, 100,
    std::bind(&CpLifecycleEventMonitor::onTransitionEvent, this, std::placeholders::_1));

  RCLCPP_INFO(
    getLogger(), "[CpLifecycleEventMonitor] Subscribed to: %s",
    (nodeName_ + node_transition_event_topic).c_str());
}

std::optional<lifecycle_msgs::msg::TransitionEvent>
CpLifecycleEventMonitor::getLastTransitionEvent() const
{
  std::lock_guard<std::mutex> lock(eventMutex_);
  if (lastTransitionEvent_)
  {
    return *lastTransitionEvent_;
  }
  return std::nullopt;
}

void CpLifecycleEventMonitor::onTransitionEvent(
  const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
  RCLCPP_INFO(
    getLogger(), "[CpLifecycleEventMonitor] Transition event: %d -> %d", msg->start_state.id,
    msg->goal_state.id);

  // Store the event
  {
    std::lock_guard<std::mutex> lock(eventMutex_);
    lastTransitionEvent_ = msg;
  }

  // Phase 3: Full state-pair matching logic (moved from ClLifecycleNode)
  // Parse transition events and emit corresponding signals

  // TRANSITION_CONFIGURE
  if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_CONFIGURE");
    onTransitionConfigure_();
    if (postEventConfigure_) postEventConfigure_();
  }
  // TRANSITION_CLEANUP
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_CLEANUP");
    onTransitionCleanup_();
    if (postEventCleanup_) postEventCleanup_();
  }
  // TRANSITION_ACTIVATE
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ACTIVATE");
    onTransitionActivate_();
    if (postEventActivate_) postEventActivate_();
  }
  // TRANSITION_DEACTIVATE
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_DEACTIVATE");
    onTransitionDeactivate_();
    if (postEventDeactivate_) postEventDeactivate_();
  }
  // TRANSITION_UNCONFIGURED_SHUTDOWN
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_UNCONFIGURED_SHUTDOWN");
    onTransitionUnconfiguredShutdown_();
    if (postEventUnconfiguredShutdown_) postEventUnconfiguredShutdown_();
  }
  // TRANSITION_INACTIVE_SHUTDOWN
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_INACTIVE_SHUTDOWN");
    onTransitionInactiveShutdown_();
    if (postEventInactiveShutdown_) postEventInactiveShutdown_();
  }
  // TRANSITION_ACTIVE_SHUTDOWN
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ACTIVE_SHUTDOWN");
    onTransitionActiveShutdown_();
    if (postEventActiveShutdown_) postEventActiveShutdown_();
  }
  // TRANSITION_DESTROY
  else if (msg->start_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_DESTROY");
    onTransitionDestroy_();
    if (postEventDestroy_) postEventDestroy_();
  }
  // TRANSITION_ON_CONFIGURE_SUCCESS
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_CONFIGURE_SUCCESS");
    onTransitionOnConfigureSuccess_();
    if (postEventOnConfigureSuccess_) postEventOnConfigureSuccess_();
  }
  // TRANSITION_ON_CONFIGURE_FAILURE
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_CONFIGURE_FAILURE");
    onTransitionOnConfigureFailure_();
    if (postEventOnConfigureFailure_) postEventOnConfigureFailure_();
  }
  // TRANSITION_ON_CONFIGURE_ERROR
  if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_CONFIGURE_ERROR");
    onTransitionOnConfigureError_();
    if (postEventOnConfigureError_) postEventOnConfigureError_();
  }
  // TRANSITION_ON_CLEANUP_SUCCESS
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_CLEANUP_SUCCESS");
    onTransitionOnCleanupSuccess_();
    if (postEventOnCleanupSuccess_) postEventOnCleanupSuccess_();
  }
  // TRANSITION_ON_CLEANUP_ERROR
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_CLEANUP_ERROR");
    onTransitionOnCleanupError_();
    if (postEventOnCleanupError_) postEventOnCleanupError_();
  }
  // TRANSITION_ON_ACTIVATE_SUCCESS
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_ACTIVATE_SUCCESS");
    onTransitionOnActivateSuccess_();
    if (postEventOnActivateSuccess_) postEventOnActivateSuccess_();
  }
  // TRANSITION_ON_ACTIVATE_FAILURE
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_ACTIVATE_FAILURE");
    onTransitionOnActivateFailure_();
    if (postEventOnActivateFailure_) postEventOnActivateFailure_();
  }
  // TRANSITION_ON_ACTIVATE_ERROR
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_ACTIVATE_ERROR");
    onTransitionOnActivateError_();
    if (postEventOnActivateError_) postEventOnActivateError_();
  }
  // TRANSITION_ON_DEACTIVATE_SUCCESS
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_DEACTIVATE_SUCCESS");
    onTransitionOnDeactivateSuccess_();
    if (postEventOnDeactivateSuccess_) postEventOnDeactivateSuccess_();
  }
  // TRANSITION_ON_DEACTIVATE_ERROR
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_DEACTIVATE_ERROR");
    onTransitionOnDeactivateError_();
    if (postEventOnDeactivateError_) postEventOnDeactivateError_();
  }
  // TRANSITION_ON_SHUTDOWN_SUCCESS
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_SHUTDOWN_SUCCESS");
    onTransitionOnShutdownSuccess_();
    if (postEventOnShutdownSuccess_) postEventOnShutdownSuccess_();
  }
  // TRANSITION_ON_SHUTDOWN_ERROR
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN &&
    msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_SHUTDOWN_ERROR");
    onTransitionOnShutdownError_();
    if (postEventOnShutdownError_) postEventOnShutdownError_();
  }
  // TRANSITION_ON_ERROR_SUCCESS
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_ERROR_SUCCESS");
    onTransitionOnErrorSuccess_();
    if (postEventOnErrorSuccess_) postEventOnErrorSuccess_();
  }
  // TRANSITION_ON_ERROR_FAILURE
  else if (
    msg->start_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING &&
    msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_ON_ERROR_FAILURE");
    onTransitionOnErrorFailure_();
    if (postEventOnErrorFailure_) postEventOnErrorFailure_();
  }
  // UNKNOWN TRANSITION
  else
  {
    RCLCPP_INFO(getLogger(), "TRANSITION_UNKNOWN");
  }
}

}  // namespace cl_lifecyclenode
