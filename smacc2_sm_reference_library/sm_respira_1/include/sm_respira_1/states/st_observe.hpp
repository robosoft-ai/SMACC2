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

namespace sm_respira_1
{
// STATE DECLARATION
struct StObserve : smacc2::SmaccState<StObserve, MsRun>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct AC_CYCLE : SUCCESS{};
  struct CMV_CYCLE : SUCCESS{};
  struct PC_CYCLE : SUCCESS{};
  struct PS_CYCLE : SUCCESS{};
  struct SHUTDOWN : SUCCESS{};
  struct CALIBRATION : SUCCESS{};

  // TRANSITION TABLE
  typedef mpl::list<

    // Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, SsACCycle, TIMEOUT>,
    // Transition<smacc2::EvTopicMessage<CbWatchdogSubscriberBehavior, OrSubscriber>, SsACCycle>,
    // Keyboard events
    Transition<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>, SsACCycle, AC_CYCLE>,
    Transition<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>, SsCMVCycle, CMV_CYCLE>,
    Transition<EvKeyPressC<CbDefaultKeyboardBehavior, OrKeyboard>, SsPCCycle, PC_CYCLE>,
    Transition<EvKeyPressD<CbDefaultKeyboardBehavior, OrKeyboard>, SsPSCycle, PS_CYCLE>,

    Transition<EvKeyPressL<CbDefaultKeyboardBehavior, OrKeyboard>, MsCalibration, CALIBRATION>,
    Transition<EvKeyPressS<CbDefaultKeyboardBehavior, OrKeyboard>, MsShutdown, SHUTDOWN>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10);
    configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
    configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_respira_1
