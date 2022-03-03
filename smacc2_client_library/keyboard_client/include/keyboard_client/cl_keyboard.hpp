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

#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <smacc2/introspection/introspection.hpp>
#include <smacc2/smacc.hpp>

#include <boost/asio.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <iostream>

#include <std_msgs/msg/u_int16.hpp>

namespace cl_keyboard
{
//----------------- TIMER sc::event DEFINITION ----------------------------------------------
template <typename TSource, typename TOrthogonal>
struct EvKeyPressA : sc::event<EvKeyPressA<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressB : sc::event<EvKeyPressB<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressC : sc::event<EvKeyPressC<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressD : sc::event<EvKeyPressD<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressE : sc::event<EvKeyPressE<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressF : sc::event<EvKeyPressF<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressG : sc::event<EvKeyPressG<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressH : sc::event<EvKeyPressH<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressI : sc::event<EvKeyPressI<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressJ : sc::event<EvKeyPressJ<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressK : sc::event<EvKeyPressK<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressL : sc::event<EvKeyPressL<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressM : sc::event<EvKeyPressM<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressN : sc::event<EvKeyPressN<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressO : sc::event<EvKeyPressO<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressP : sc::event<EvKeyPressP<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressQ : sc::event<EvKeyPressQ<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressR : sc::event<EvKeyPressR<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressS : sc::event<EvKeyPressS<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressT : sc::event<EvKeyPressT<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressU : sc::event<EvKeyPressU<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressV : sc::event<EvKeyPressV<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressW : sc::event<EvKeyPressW<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressX : sc::event<EvKeyPressX<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressY : sc::event<EvKeyPressY<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvKeyPressZ : sc::event<EvKeyPressZ<TSource, TOrthogonal>>
{
};

//------------------  KEYBOARD CLIENT ---------------------------------------------

class ClKeyboard : public smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::UInt16>
{
public:
  ClKeyboard();
  virtual ~ClKeyboard();
  virtual void onInitialize() override;

  smacc2::SmaccSignal<void(char keypress)> OnKeyPress_;

  template <typename T>
  void OnKeyPress(void (T::*callback)(char keypress), T * object)
  {
    //this->connectSignal(OnKeyPress_, callback, object);
    this->getStateMachine()->createSignalConnection(OnKeyPress_, callback, object);
  }

  std::function<void(std_msgs::msg::UInt16)> postEventKeyPress;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    // call to the base configuration to properly handling the message and initial message smacc events
    smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::UInt16>::onOrthogonalAllocation<
      TOrthogonal, TSourceObject>();

    postEventKeyPress = [=](auto unicode_keychar) {
      char character = (char)unicode_keychar.data;
      RCLCPP_WARN(getLogger(), "detected keyboard: %c", character);

      if (character == 'a')
        this->postKeyEvent<EvKeyPressA<ClKeyboard, TOrthogonal>>();
      else if (character == 'b')
        this->postKeyEvent<EvKeyPressB<ClKeyboard, TOrthogonal>>();
      else if (character == 'c')
        this->postKeyEvent<EvKeyPressC<ClKeyboard, TOrthogonal>>();
      else if (character == 'd')
        this->postKeyEvent<EvKeyPressD<ClKeyboard, TOrthogonal>>();
      else if (character == 'e')
        this->postKeyEvent<EvKeyPressE<ClKeyboard, TOrthogonal>>();
      else if (character == 'f')
        this->postKeyEvent<EvKeyPressF<ClKeyboard, TOrthogonal>>();
      else if (character == 'g')
        this->postKeyEvent<EvKeyPressG<ClKeyboard, TOrthogonal>>();
      else if (character == 'h')
        this->postKeyEvent<EvKeyPressH<ClKeyboard, TOrthogonal>>();
      else if (character == 'y')
        this->postKeyEvent<EvKeyPressI<ClKeyboard, TOrthogonal>>();
      else if (character == 'j')
        this->postKeyEvent<EvKeyPressJ<ClKeyboard, TOrthogonal>>();
      else if (character == 'k')
        this->postKeyEvent<EvKeyPressK<ClKeyboard, TOrthogonal>>();
      else if (character == 'l')
        this->postKeyEvent<EvKeyPressL<ClKeyboard, TOrthogonal>>();
      else if (character == 'm')
        this->postKeyEvent<EvKeyPressM<ClKeyboard, TOrthogonal>>();
      else if (character == 'n')
        this->postKeyEvent<EvKeyPressN<ClKeyboard, TOrthogonal>>();
      else if (character == 'o')
        this->postKeyEvent<EvKeyPressO<ClKeyboard, TOrthogonal>>();
      else if (character == 'p')
        this->postKeyEvent<EvKeyPressP<ClKeyboard, TOrthogonal>>();
      else if (character == 'q')
        this->postKeyEvent<EvKeyPressQ<ClKeyboard, TOrthogonal>>();
      else if (character == 'r')
        this->postKeyEvent<EvKeyPressR<ClKeyboard, TOrthogonal>>();
      else if (character == 's')
        this->postKeyEvent<EvKeyPressS<ClKeyboard, TOrthogonal>>();
      else if (character == 't')
        this->postKeyEvent<EvKeyPressT<ClKeyboard, TOrthogonal>>();
      else if (character == 'u')
        this->postKeyEvent<EvKeyPressU<ClKeyboard, TOrthogonal>>();
      else if (character == 'v')
        this->postKeyEvent<EvKeyPressV<ClKeyboard, TOrthogonal>>();
      else if (character == 'w')
        this->postKeyEvent<EvKeyPressW<ClKeyboard, TOrthogonal>>();
      else if (character == 'x')
        this->postKeyEvent<EvKeyPressX<ClKeyboard, TOrthogonal>>();
      else if (character == 'y')
        this->postKeyEvent<EvKeyPressY<ClKeyboard, TOrthogonal>>();
      else if (character == 'z')
        this->postKeyEvent<EvKeyPressZ<ClKeyboard, TOrthogonal>>();
      OnKeyPress_(character); /*  */
    };
  }

  void onKeyboardMessage(const std_msgs::msg::UInt16 & unicode_keychar);

  template <typename TEv>
  void postKeyEvent()
  {
    RCLCPP_WARN(
      getLogger(), "ClKeyboard ev: %s", smacc2::demangleSymbol(typeid(TEv).name()).c_str());
    this->postEvent<TEv>();
  }

private:
  bool initialized_;
};
}  // namespace cl_keyboard
