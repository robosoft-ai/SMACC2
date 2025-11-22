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

#pragma once

#include <smacc2/component.hpp>
#include <smacc2/smacc_signal.hpp>

#include <lifecycle_msgs/msg/transition_event.hpp>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <mutex>
#include <optional>
#include <string>

// Forward declarations
namespace cl_lifecyclenode
{
class CpLifecycleEventMonitor;

// Event declarations - component is now the source
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionConfigure;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionActivate;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionDeactivate;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionCleanup;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionUnconfiguredShutdown;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionInactiveShutdown;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionActiveShutdown;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionDestroy;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnConfigureSuccess;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnConfigureFailure;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnConfigureError;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnActivateSuccess;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnActivateFailure;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnActivateError;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnDeactivateSuccess;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnDeactivateFailure;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnDeactivateError;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnCleanupSuccess;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnCleanupFailure;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnCleanupError;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnShutdownSuccess;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnShutdownFailure;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnShutdownError;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnErrorSuccess;
template <typename TSourceObject, typename TOrthogonal>
struct EvTransitionOnErrorFailure;
}  // namespace cl_lifecyclenode

namespace cl_lifecyclenode
{
/**
 * @brief Component that monitors ROS2 lifecycle node transition events
 *
 * CpLifecycleEventMonitor subscribes to lifecycle transition events and parses
 * them to detect specific transition types. It emits SMACC2 signals for each
 * transition, enabling event-driven responses to lifecycle state changes.
 *
 * This component handles the complex state-pair matching logic required to
 * distinguish between transition starts, successes, failures, and errors.
 */
class CpLifecycleEventMonitor : public smacc2::ISmaccComponent
{
public:
  /**
   * @brief Constructor with node name
   * @param nodeName Name of the lifecycle node to monitor
   */
  CpLifecycleEventMonitor(std::string nodeName);

  virtual ~CpLifecycleEventMonitor() = default;

  /**
   * @brief Set the owner client and state machine for this component
   * @param owner Pointer to the owning client
   * @param stateMachine Pointer to the state machine
   */
  void setOwner(smacc2::ISmaccClient * owner, smacc2::ISmaccStateMachine * stateMachine)
  {
    owner_ = owner;
    stateMachine_ = stateMachine;
  }

  // SMACC2 Signals for lifecycle transition events
  // Transition initiation signals
  smacc2::SmaccSignal<void()> onTransitionCreate_;
  smacc2::SmaccSignal<void()> onTransitionConfigure_;
  smacc2::SmaccSignal<void()> onTransitionActivate_;
  smacc2::SmaccSignal<void()> onTransitionDeactivate_;
  smacc2::SmaccSignal<void()> onTransitionCleanup_;
  smacc2::SmaccSignal<void()> onTransitionUnconfiguredShutdown_;
  smacc2::SmaccSignal<void()> onTransitionInactiveShutdown_;
  smacc2::SmaccSignal<void()> onTransitionActiveShutdown_;
  smacc2::SmaccSignal<void()> onTransitionDestroy_;

  // Configure transition result signals
  smacc2::SmaccSignal<void()> onTransitionOnConfigureSuccess_;
  smacc2::SmaccSignal<void()> onTransitionOnConfigureFailure_;
  smacc2::SmaccSignal<void()> onTransitionOnConfigureError_;

  // Activate transition result signals
  smacc2::SmaccSignal<void()> onTransitionOnActivateSuccess_;
  smacc2::SmaccSignal<void()> onTransitionOnActivateFailure_;
  smacc2::SmaccSignal<void()> onTransitionOnActivateError_;

  // Deactivate transition result signals
  smacc2::SmaccSignal<void()> onTransitionOnDeactivateSuccess_;
  smacc2::SmaccSignal<void()> onTransitionOnDeactivateFailure_;
  smacc2::SmaccSignal<void()> onTransitionOnDeactivateError_;

  // Cleanup transition result signals
  smacc2::SmaccSignal<void()> onTransitionOnCleanupSuccess_;
  smacc2::SmaccSignal<void()> onTransitionOnCleanupFailure_;
  smacc2::SmaccSignal<void()> onTransitionOnCleanupError_;

  // Shutdown transition result signals
  smacc2::SmaccSignal<void()> onTransitionOnShutdownSuccess_;
  smacc2::SmaccSignal<void()> onTransitionOnShutdownFailure_;
  smacc2::SmaccSignal<void()> onTransitionOnShutdownError_;

  // Active shutdown specific signals
  smacc2::SmaccSignal<void()> onTransitionOnActiveShutdownSuccess_;
  smacc2::SmaccSignal<void()> onTransitionOnActiveShutdownFailure_;
  smacc2::SmaccSignal<void()> onTransitionOnActiveShutdownError_;

  // Error processing result signals
  smacc2::SmaccSignal<void()> onTransitionOnErrorSuccess_;
  smacc2::SmaccSignal<void()> onTransitionOnErrorFailure_;
  smacc2::SmaccSignal<void()> onTransitionOnErrorError_;

  /**
   * @brief Get the last received transition event
   * @return Optional transition event (empty if none received)
   */
  std::optional<lifecycle_msgs::msg::TransitionEvent> getLastTransitionEvent() const;

  /**
   * @brief Component initialization - creates subscription
   */
  void onInitialize() override;

  /**
   * @brief Configure event posting with orthogonal template parameter
   * This is called from the client's onStateOrthogonalAllocation to set up type-safe event posting
   */
  template <typename TOrthogonal, typename TClient>
  void onStateOrthogonalAllocation()
  {
    // Set up event posting lambdas with correct template parameters
    // Component is the event source (TSourceObject = CpLifecycleEventMonitor)

    postEventConfigure_ = [this]()
    { this->postEvent<EvTransitionConfigure<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventActivate_ = [this]()
    { this->postEvent<EvTransitionActivate<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventDeactivate_ = [this]()
    { this->postEvent<EvTransitionDeactivate<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventCleanup_ = [this]()
    { this->postEvent<EvTransitionCleanup<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventUnconfiguredShutdown_ = [this]()
    { this->postEvent<EvTransitionUnconfiguredShutdown<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventInactiveShutdown_ = [this]()
    { this->postEvent<EvTransitionInactiveShutdown<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventActiveShutdown_ = [this]()
    { this->postEvent<EvTransitionActiveShutdown<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventDestroy_ = [this]()
    { this->postEvent<EvTransitionDestroy<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnConfigureSuccess_ = [this]()
    { this->postEvent<EvTransitionOnConfigureSuccess<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnConfigureFailure_ = [this]()
    { this->postEvent<EvTransitionOnConfigureFailure<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnConfigureError_ = [this]()
    { this->postEvent<EvTransitionOnConfigureError<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnActivateSuccess_ = [this]()
    { this->postEvent<EvTransitionOnActivateSuccess<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnActivateFailure_ = [this]()
    { this->postEvent<EvTransitionOnActivateFailure<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnActivateError_ = [this]()
    { this->postEvent<EvTransitionOnActivateError<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnDeactivateSuccess_ = [this]()
    { this->postEvent<EvTransitionOnDeactivateSuccess<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnDeactivateFailure_ = [this]()
    { this->postEvent<EvTransitionOnDeactivateFailure<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnDeactivateError_ = [this]()
    { this->postEvent<EvTransitionOnDeactivateError<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnCleanupSuccess_ = [this]()
    { this->postEvent<EvTransitionOnCleanupSuccess<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnCleanupFailure_ = [this]()
    { this->postEvent<EvTransitionOnCleanupFailure<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnCleanupError_ = [this]()
    { this->postEvent<EvTransitionOnCleanupError<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnShutdownSuccess_ = [this]()
    { this->postEvent<EvTransitionOnShutdownSuccess<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnShutdownFailure_ = [this]()
    { this->postEvent<EvTransitionOnShutdownFailure<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnShutdownError_ = [this]()
    { this->postEvent<EvTransitionOnShutdownError<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnErrorSuccess_ = [this]()
    { this->postEvent<EvTransitionOnErrorSuccess<CpLifecycleEventMonitor, TOrthogonal>>(); };

    postEventOnErrorFailure_ = [this]()
    { this->postEvent<EvTransitionOnErrorFailure<CpLifecycleEventMonitor, TOrthogonal>>(); };
  }

private:
  std::string nodeName_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr subscription_;
  lifecycle_msgs::msg::TransitionEvent::SharedPtr lastTransitionEvent_;
  mutable std::mutex eventMutex_;

  /**
   * @brief Callback for transition event subscription
   * @param msg Transition event message
   */
  void onTransitionEvent(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);

  // Event posting lambdas - configured via onStateOrthogonalAllocation<TOrthogonal, TClient>()
  std::function<void()> postEventConfigure_;
  std::function<void()> postEventActivate_;
  std::function<void()> postEventDeactivate_;
  std::function<void()> postEventCleanup_;
  std::function<void()> postEventUnconfiguredShutdown_;
  std::function<void()> postEventInactiveShutdown_;
  std::function<void()> postEventActiveShutdown_;
  std::function<void()> postEventDestroy_;

  std::function<void()> postEventOnConfigureSuccess_;
  std::function<void()> postEventOnConfigureFailure_;
  std::function<void()> postEventOnConfigureError_;

  std::function<void()> postEventOnActivateSuccess_;
  std::function<void()> postEventOnActivateFailure_;
  std::function<void()> postEventOnActivateError_;

  std::function<void()> postEventOnDeactivateSuccess_;
  std::function<void()> postEventOnDeactivateFailure_;
  std::function<void()> postEventOnDeactivateError_;

  std::function<void()> postEventOnCleanupSuccess_;
  std::function<void()> postEventOnCleanupFailure_;
  std::function<void()> postEventOnCleanupError_;

  std::function<void()> postEventOnShutdownSuccess_;
  std::function<void()> postEventOnShutdownFailure_;
  std::function<void()> postEventOnShutdownError_;

  std::function<void()> postEventOnErrorSuccess_;
  std::function<void()> postEventOnErrorFailure_;
};

}  // namespace cl_lifecyclenode
