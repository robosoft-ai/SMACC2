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

#include <keyboard_client/cl_keyboard.hpp>

namespace cl_keyboard
{
ClKeyboard::ClKeyboard()
{
  initialized_ = false;
  topicName = "/keyboard_unicode";
}

ClKeyboard::~ClKeyboard() {}

void ClKeyboard::onInitialize()
{
  SmaccSubscriberClient<std_msgs::msg::UInt16>::onInitialize();

  if (!this->initialized_)
  {
    this->onMessageReceived(&ClKeyboard::onKeyboardMessage, this);
    this->initialized_ = true;
  }
}

void ClKeyboard::onKeyboardMessage(const std_msgs::msg::UInt16 & unicode_keychar)
{
  postEventKeyPress(unicode_keychar);
}
}  // namespace cl_keyboard
