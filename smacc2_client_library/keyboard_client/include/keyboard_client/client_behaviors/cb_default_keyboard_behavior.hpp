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

#include <keyboard_client/cl_keyboard.hpp>
#include <smacc2/smacc_client_behavior.hpp>

#include <std_msgs/msg/u_int16.hpp>

namespace cl_keyboard
{
class CbDefaultKeyboardBehavior : public smacc2::SmaccClientBehavior
{
public:
  ClKeyboard * ClKeyboard_;
  std::function<void(char)> postEventKeyPress;

  void onEntry();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    postEventKeyPress = [=](char character) {
      if (character == 'a')
        postKeyEvent<EvKeyPressA<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'b')
        postKeyEvent<EvKeyPressB<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'c')
        postKeyEvent<EvKeyPressC<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'd')
        postKeyEvent<EvKeyPressD<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'e')
        postKeyEvent<EvKeyPressE<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'f')
        postKeyEvent<EvKeyPressF<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'g')
        postKeyEvent<EvKeyPressG<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'h')
        postKeyEvent<EvKeyPressH<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'y')
        postKeyEvent<EvKeyPressI<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'j')
        postKeyEvent<EvKeyPressJ<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'k')
        postKeyEvent<EvKeyPressK<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'l')
        postKeyEvent<EvKeyPressL<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'm')
        postKeyEvent<EvKeyPressM<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'n')
        postKeyEvent<EvKeyPressN<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'o')
        postKeyEvent<EvKeyPressO<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'p')
        postKeyEvent<EvKeyPressP<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'q')
        postKeyEvent<EvKeyPressQ<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'r')
        postKeyEvent<EvKeyPressR<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 's')
        postKeyEvent<EvKeyPressS<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 't')
        postKeyEvent<EvKeyPressT<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'u')
        postKeyEvent<EvKeyPressU<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'v')
        postKeyEvent<EvKeyPressV<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'w')
        postKeyEvent<EvKeyPressW<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'x')
        postKeyEvent<EvKeyPressX<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'y')
        postKeyEvent<EvKeyPressY<CbDefaultKeyboardBehavior, TOrthogonal>>();
      else if (character == 'z')
        postKeyEvent<EvKeyPressZ<CbDefaultKeyboardBehavior, TOrthogonal>>();
    };
  }

  void OnKeyPress(char character);

  template <typename TEv>
  void postKeyEvent()
  {
    RCLCPP_WARN(
      getNode()->get_logger(), "CbDefaultKeyboardBehavior %ld ev: %s", (long)(void *)this,
      smacc2::demangleSymbol(typeid(TEv).name()).c_str());
    auto event = new TEv();
    this->postEvent(event);
  }
};
}  // namespace cl_keyboard
