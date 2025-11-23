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

#include <cl_lifecycle_node/components/cp_lifecycle_event_monitor.hpp>
#include <lifecycle_msgs/msg/transition_event.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <smacc2/client_core_components/cp_service_client.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_client_behavior_base.hpp>

namespace cl_lifecycle_node
{
template <typename TSourceObject, typename TOrthogonal>
struct EvLifecycleNodeChangeState
: sc::event<EvLifecycleNodeChangeState<TSourceObject, TOrthogonal>>
{
  uint8_t transition;
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionCreate : public sc::event<EvTransitionCreate<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionConfigure : public sc::event<EvTransitionConfigure<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionCleanup : sc::event<EvTransitionCleanup<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionActivate : sc::event<EvTransitionActivate<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionDeactivate : sc::event<EvTransitionDeactivate<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionUnconfiguredShutdown
: sc::event<EvTransitionUnconfiguredShutdown<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionInactiveShutdown
: sc::event<EvTransitionInactiveShutdown<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionActiveShutdown
: sc::event<EvTransitionActiveShutdown<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionDestroy : sc::event<EvTransitionDestroy<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnConfigureSuccess
: sc::event<EvTransitionOnConfigureSuccess<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnConfigureFailure
: sc::event<EvTransitionOnConfigureFailure<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnConfigureError
: sc::event<EvTransitionOnConfigureError<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnActivateSuccess
: sc::event<EvTransitionOnActivateSuccess<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnActivateFailure
: sc::event<EvTransitionOnActivateFailure<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnActivateError
: sc::event<EvTransitionOnActivateError<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnDeactivateSuccess
: sc::event<EvTransitionOnDeactivateSuccess<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnDeactivateFailure
: sc::event<EvTransitionOnDeactivateFailure<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnDeactivateError
: sc::event<EvTransitionOnDeactivateError<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnCleanupSuccess
: sc::event<EvTransitionOnCleanupSuccess<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnCleanupFailure
: sc::event<EvTransitionOnCleanupFailure<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnCleanupError
: sc::event<EvTransitionOnCleanupError<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnShutdownSuccess
: sc::event<EvTransitionOnShutdownSuccess<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnShutdownFailure
: sc::event<EvTransitionOnShutdownFailure<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnShutdownError
: sc::event<EvTransitionOnShutdownError<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnErrorSuccess
: sc::event<EvTransitionOnErrorSuccess<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnErrorFailure
: sc::event<EvTransitionOnErrorFailure<TSourceObject, TOrthogonal>>
{
};

template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnErrorError : sc::event<EvTransitionOnErrorError<TSourceObject, TOrthogonal>>
{
};

class ClLifecycleNode : public smacc2::ISmaccClient
{
public:
  ClLifecycleNode(std::string nodeName);

  virtual ~ClLifecycleNode();

  void onInitialize() override;

  // @brief execute this method to trigger a transition to some state
  void changeState(uint8_t state);

  // @brief execute this method to trigger the configure transition
  void configure();

  // @brief execute this method to trigger the activate transition
  void activate();

  // @brief execute this method to trigger the deactivate transition
  void deactivate();

  // @brief execute this method to trigger the cleanup transition
  void cleanup();

  // @brief execute this method to trigger the shutdown transition
  void shutdown();

  // @brief execute this method to trigger the destroy transition
  void destroy();

  /**
   * @brief Configure component for event posting during orthogonal allocation
   */
  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    // Create and register component with orthogonal
    if (!eventMonitor_)
    {
      eventMonitor_ =
        this->template createComponent<CpLifecycleEventMonitor, TOrthogonal, ClLifecycleNode>(
          nodeName_);
    }

    // Configure component for this orthogonal
    eventMonitor_->template onStateOrthogonalAllocation<TOrthogonal, ClLifecycleNode>();
  }

private:
  // Phase 2: Service clients (will be migrated to CpServiceClient components in future enhancement)
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_get_state_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_change_state_;

  std::string nodeName_;

  // Phase 3: Event monitor component
  CpLifecycleEventMonitor * eventMonitor_;
  const std::string node_get_state_topic = "/get_state";
  const std::string node_change_state_topic = "/change_state";

  lifecycle_msgs::msg::TransitionEvent::SharedPtr lastTransitionEvent_;
};

}  // namespace cl_lifecycle_node
