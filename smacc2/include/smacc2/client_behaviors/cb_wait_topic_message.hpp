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

#pragma once

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <smacc2/smacc_signal.hpp>

namespace smacc2
{
namespace client_behaviors
{
using namespace std::chrono_literals;

// Asynchronous behavior that waits to a topic message to send EvCbSuccess event
// a guard function can be set to use conditions on the contents
template <typename TMessage>
class CbWaitTopicMessage : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbWaitTopicMessage(
    const char * topicname, std::function<bool(const TMessage &)> guardFunction = nullptr)
  {
    topicname_ = topicname;
    guardFn_ = guardFunction;
  }

  virtual ~CbWaitTopicMessage() {}

  // On the state machine thread, before onEntry: the slot goes through the state-lifetime signal
  // map, so dispose finalizes it and waits for a callback in flight before this behavior dies.
  void runtimeConfigure() override
  {
    signal_ = std::make_shared<MessageSignal>();
    this->getStateMachine()->createSignalConnection(
      *signal_, &CbWaitTopicMessage::onMessageReceived, this);
  }

  void onEntry() override
  {
    rclcpp::SensorDataQoS qos;

    RCLCPP_INFO_STREAM(
      getLogger(), "[CbWaitTopicMessage] waiting message from topic: "
                     << topicname_ << "[" << demangledTypeName<TMessage>() << "]");

    // The subscription callback co-owns the signal and touches nothing of this behavior; after
    // dispose the signal has no slot and a late message does nothing.
    auto signal = signal_;
    std::function<void(typename TMessage::SharedPtr)> fn = [signal](auto msg) { (*signal)(*msg); };

    auto nh = getNode();
    sub_ = nh->template create_subscription<TMessage>(topicname_, qos, fn);
  }

  void onMessageReceived(const TMessage & msg)
  {
    if (!success && (guardFn_ == nullptr || guardFn_(msg)))
    {
      RCLCPP_INFO(getLogger(), "[CbWaitTopicMessage] message received.");
      success = true;
      this->postSuccessEvent();
    }
  }

protected:
  using MessageSignal = smacc2::SmaccSignal<void(const TMessage &)>;

  bool success = false;
  std::shared_ptr<MessageSignal> signal_;
  typename rclcpp::Subscription<TMessage>::SharedPtr sub_;
  std::function<bool(const TMessage &)> guardFn_;
  std::string topicname_;
};
}  // namespace client_behaviors
}  // namespace smacc2
