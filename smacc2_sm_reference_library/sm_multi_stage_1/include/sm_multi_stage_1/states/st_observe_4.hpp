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

namespace sm_multi_stage_1
{
// STATE DECLARATION
struct StObserve4 : smacc2::SmaccState<StObserve4, MsRun4>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct ac_cycle_4 : SUCCESS{};
  struct dc_cycle_4 : SUCCESS{};
  struct gc_cycle_4 : SUCCESS{};
  struct cmv_cycle_4 : SUCCESS{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>, ACCycleLoop4, SUCCESS>,
    Transition<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>, CMVCycleLoop4, SUCCESS>,
    Transition<EvKeyPressD<CbDefaultKeyboardBehavior, OrKeyboard>, DCCycleLoop4, SUCCESS>,
    Transition<EvKeyPressG<CbDefaultKeyboardBehavior, OrKeyboard>, GCCycleLoop4, SUCCESS>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, GCCycleLoop4, SUCCESS>

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
}  // namespace sm_multi_stage_1
