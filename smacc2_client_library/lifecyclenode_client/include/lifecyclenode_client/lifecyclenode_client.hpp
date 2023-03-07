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
#include <smacc2/smacc_client_behavior_base.hpp>

namespace cl_lifecyclenode
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

  // @brief execute this method to trigger the create transition
  virtual void onTransitionEvent(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);

  boost::signals2::signal<void(const lifecycle_msgs::msg::TransitionEvent::SharedPtr)>
    onTransitionEventSignal;

  boost::signals2::signal<void()> onTransitionCreate_;
  boost::signals2::signal<void()> onTransitionConfigure_;
  boost::signals2::signal<void()> onTransitionCleanup_;
  boost::signals2::signal<void()> onTransitionActivate_;
  boost::signals2::signal<void()> onTransitionDeactivate_;

  boost::signals2::signal<void()> onTransitionUnconfiguredShutdown_;
  boost::signals2::signal<void()> onTransitionInactiveShutdown_;
  boost::signals2::signal<void()> onTransitionActiveShutdown_;

  boost::signals2::signal<void()> onTransitionDestroy_;

  boost::signals2::signal<void()> onTransitionOnConfigureSuccess_;
  boost::signals2::signal<void()> onTransitionOnConfigureFailure_;
  boost::signals2::signal<void()> onTransitionOnConfigureError_;

  boost::signals2::signal<void()> onTransitionOnActivateSuccess_;
  boost::signals2::signal<void()> onTransitionOnActivateFailure_;
  boost::signals2::signal<void()> onTransitionOnActivateError_;

  boost::signals2::signal<void()> onTransitionOnDeactivateSuccess_;
  boost::signals2::signal<void()> onTransitionOnDeactivateFailure_;
  boost::signals2::signal<void()> onTransitionOnDeactivateError_;

  boost::signals2::signal<void()> onTransitionOnCleanupSuccess_;
  boost::signals2::signal<void()> onTransitionOnCleanupFailure_;
  boost::signals2::signal<void()> onTransitionOnCleanupError_;

  boost::signals2::signal<void()> onTransitionOnShutdownSuccess_;
  boost::signals2::signal<void()> onTransitionOnShutdownFailure_;
  boost::signals2::signal<void()> onTransitionOnShutdownError_;

  boost::signals2::signal<void()> onTransitionOnActiveShutdownSuccess_;
  boost::signals2::signal<void()> onTransitionOnActiveShutdownFailure_;
  boost::signals2::signal<void()> onTransitionOnActiveShutdownError_;

  boost::signals2::signal<void()> onTransitionOnErrorSuccess_;
  boost::signals2::signal<void()> onTransitionOnErrorFailure_;
  boost::signals2::signal<void()> onTransitionOnErrorError_;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    this->postOnTransitionCreate_ = [=]()
    {
      RCLCPP_INFO(this->getLogger(), "postOnTransitionCreate_");
      this->onTransitionCreate_();
      sc::event<EvTransitionCreate<TSourceObject, TOrthogonal>> * ev =
        new EvTransitionCreate<TSourceObject, TOrthogonal>();
      this->postEvent(ev);
    };

    this->postOnTransitionConfigure_ = [=]()
    {
      this->onTransitionConfigure_();
      this->postEvent<EvTransitionConfigure<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionActivate_ = [=]()
    {
      this->onTransitionActivate_();
      this->postEvent<EvTransitionActivate<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionDeactivate_ = [=]()
    {
      this->onTransitionDeactivate_();
      this->postEvent<EvTransitionDeactivate<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionUnconfiguredShutdown_ = [=]()
    {
      this->onTransitionUnconfiguredShutdown_();
      this->postEvent<EvTransitionUnconfiguredShutdown<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionInactiveShutdown_ = [=]()
    {
      this->onTransitionInactiveShutdown_();
      this->postEvent<EvTransitionInactiveShutdown<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionActiveShutdown_ = [=]()
    {
      this->onTransitionActiveShutdown_();
      this->postEvent<EvTransitionActiveShutdown<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionDestroy_ = [=]()
    {
      this->onTransitionDestroy_();
      this->postEvent<EvTransitionDestroy<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnConfigureSuccess_ = [=]()
    {
      this->onTransitionOnConfigureSuccess_();
      this->postEvent<EvTransitionOnConfigureSuccess<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnConfigureFailure_ = [=]()
    {
      this->onTransitionOnConfigureFailure_();
      this->postEvent<EvTransitionOnConfigureFailure<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnConfigureError_ = [=]()
    {
      this->onTransitionOnConfigureError_();
      this->postEvent<EvTransitionOnConfigureError<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnActivateSuccess_ = [=]()
    {
      this->onTransitionOnActivateSuccess_();
      this->postEvent<EvTransitionOnActivateSuccess<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnActivateFailure_ = [=]()
    {
      this->onTransitionOnActivateFailure_();
      this->postEvent<EvTransitionOnActivateFailure<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnActivateError_ = [=]()
    {
      this->onTransitionOnActivateError_();
      this->postEvent<EvTransitionOnActivateError<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnDeactivateSuccess_ = [=]()
    {
      this->onTransitionOnDeactivateSuccess_();
      this->postEvent<EvTransitionOnDeactivateSuccess<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnDeactivateFailure_ = [=]()
    {
      this->onTransitionOnDeactivateFailure_();
      this->postEvent<EvTransitionOnDeactivateFailure<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnDeactivateError_ = [=]()
    {
      this->onTransitionOnDeactivateError_();
      this->postEvent<EvTransitionOnDeactivateError<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionCleanup_ = [=]()
    {
      this->onTransitionCleanup_();
      this->postEvent<EvTransitionCleanup<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnCleanupSuccess_ = [=]()
    {
      this->onTransitionOnCleanupSuccess_();
      this->postEvent<EvTransitionOnCleanupSuccess<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnCleanupFailure_ = [=]()
    {
      this->onTransitionOnCleanupFailure_();
      this->postEvent<EvTransitionOnCleanupFailure<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnCleanupError_ = [=]()
    {
      this->onTransitionOnCleanupError_();
      this->postEvent<EvTransitionOnCleanupError<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnShutdownSuccess_ = [=]()
    {
      this->onTransitionOnShutdownSuccess_();
      this->postEvent<EvTransitionOnShutdownSuccess<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnShutdownFailure_ = [=]()
    {
      this->onTransitionOnShutdownFailure_();
      this->postEvent<EvTransitionOnShutdownFailure<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnShutdownError_ = [=]()
    {
      this->onTransitionOnShutdownError_();
      this->postEvent<EvTransitionOnShutdownError<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnErrorSuccess_ = [=]()
    {
      this->onTransitionOnErrorSuccess_();
      this->postEvent<EvTransitionOnErrorSuccess<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnErrorFailure_ = [=]()
    {
      this->onTransitionOnErrorFailure_();
      this->postEvent<EvTransitionOnErrorFailure<TSourceObject, TOrthogonal>>();
    };

    this->postOnTransitionOnErrorError_ = [=]()
    {
      this->onTransitionOnErrorError_();
      this->postEvent<EvTransitionOnErrorError<TSourceObject, TOrthogonal>>();
    };
  }

private:
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_get_state_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_change_state_;

  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr
    subscription_transition_event_;

  std::string nodeName_;
  const std::string node_get_state_topic = "/get_state";
  const std::string node_change_state_topic = "/change_state";
  const std::string node_transition_event_topic = "/transition_event";

  lifecycle_msgs::msg::TransitionEvent::SharedPtr lastTransitionEvent_;

  std::function<void()> postOnTransitionCreate_;
  std::function<void()> postOnTransitionConfigure_;
  std::function<void()> postOnTransitionCleanup_;
  std::function<void()> postOnTransitionActivate_;
  std::function<void()> postOnTransitionDeactivate_;
  std::function<void()> postOnTransitionUnconfiguredShutdown_;
  std::function<void()> postOnTransitionInactiveShutdown_;
  std::function<void()> postOnTransitionActiveShutdown_;
  std::function<void()> postOnTransitionDestroy_;

  std::function<void()> postOnTransitionOnConfigureSuccess_;
  std::function<void()> postOnTransitionOnConfigureFailure_;
  std::function<void()> postOnTransitionOnConfigureError_;

  std::function<void()> postOnTransitionOnActivateSuccess_;
  std::function<void()> postOnTransitionOnActivateFailure_;
  std::function<void()> postOnTransitionOnActivateError_;

  std::function<void()> postOnTransitionOnDeactivateSuccess_;
  std::function<void()> postOnTransitionOnDeactivateFailure_;
  std::function<void()> postOnTransitionOnDeactivateError_;

  std::function<void()> postOnTransitionOnCleanupSuccess_;
  std::function<void()> postOnTransitionOnCleanupFailure_;
  std::function<void()> postOnTransitionOnCleanupError_;

  std::function<void()> postOnTransitionOnShutdownSuccess_;
  std::function<void()> postOnTransitionOnShutdownFailure_;
  std::function<void()> postOnTransitionOnShutdownError_;

  std::function<void()> postOnTransitionOnErrorSuccess_;
  std::function<void()> postOnTransitionOnErrorFailure_;
  std::function<void()> postOnTransitionOnErrorError_;
};

}  // namespace cl_lifecyclenode
