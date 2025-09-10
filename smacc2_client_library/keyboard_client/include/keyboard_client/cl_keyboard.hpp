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

#include <keyboard_client/components/cp_keyboard_listener_1.hpp>
#include <smacc2/client_base_components/cp_topic_subscriber.hpp>

#include <smacc2/introspection/introspection.hpp>
#include <smacc2/smacc.hpp>

#include <boost/asio.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <iostream>

#include <std_msgs/msg/u_int16.hpp>

namespace cl_keyboard
{
//------------------  KEYBOARD CLIENT ---------------------------------------------

class ClKeyboard : public smacc2::ISmaccClient
{
public:
  ClKeyboard();
  virtual ~ClKeyboard();

  // Override the base class methods to call our setup
  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  // clients utilizes a composition based architecture for their components
  // here we define the list of components that this client will have in a component based architecture
  {
    // for listener we use dependency injection pattern where we reference the CpTopicSubscriber inside the smacc core
    // this would be the basic subscription component to the topic
    // we use this to gain the topic funcionality interated with SMACC and that post smacc events for transitions
    // we are using it to handle ros topic messages reception and notifying other components in the client
    this->createComponent<
      smacc2::components::CpTopicSubscriber<std_msgs::msg::UInt16>, TOrthogonal, ClKeyboard>(
      "/keyboard_unicode");

    // this keyboard subscriber component requires the first subscriber component
    // it is notified by the CpTopicSubscriber and processes the messages to decide with keyboard event must be posted and then post it
    this->createComponent<cl_keyboard::components::CpKeyboardListener1, TOrthogonal, ClKeyboard>();
  }
};
}  // namespace cl_keyboard
