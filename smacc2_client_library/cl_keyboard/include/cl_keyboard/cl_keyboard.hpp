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

#include <cl_keyboard/components/cp_keyboard_listener_1.hpp>

#include <smacc2/client_core_components/cp_topic_subscriber.hpp>

#include <smacc2/introspection/introspection.hpp>
#include <smacc2/smacc.hpp>

#include <boost/asio.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <iostream>

#include <std_msgs/msg/u_int16.hpp>

namespace cl_keyboard
{
class ClKeyboard : public smacc2::ISmaccClient
{
public:
  ClKeyboard();
  virtual ~ClKeyboard();

  // Override the base class methods to call our setup
  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()

  // Clients utilize a composition based architecture for their components.
  // In the function body below we create the components that will be used in this client.

  {
    // We start by creating a topic subscriber component from SMACC2s client core components.
    // We use this to gain the topic funcionality interated with SMACC and the ability to post smacc events for transitions.
    // We are using it to handle the reception of ros topic messages and to notify the other components in the client.
    this->createComponent<
      smacc2::client_core_components::CpTopicSubscriber<std_msgs::msg::UInt16>, TOrthogonal,
      ClKeyboard>("/keyboard_unicode");

    // This keyboard listener component requires CpTopicSubscriber.
    // It is notified by the CpTopicSubscriber and processes the messages to decide which keyboard event must be posted and then posts it.
    this->createComponent<cl_keyboard::components::CpKeyboardListener1, TOrthogonal, ClKeyboard>();
  }
};
}  // namespace cl_keyboard
