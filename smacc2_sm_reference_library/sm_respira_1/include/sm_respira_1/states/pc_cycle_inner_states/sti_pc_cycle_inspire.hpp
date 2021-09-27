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
namespace pc_cycle_inner_states
{
// STATE DECLARATION
struct StiPCCycleInspire : smacc2::SmaccState<StiPCCycleInspire, SsPCCycle>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : ABORT{};
  struct NEXT : SUCCESS{};
  struct PREVIOUS : ABORT{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StiPCCyclePlateau, TIMEOUT>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StiPCCyclePlateau, NEXT>,
    Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StiPCCycleLoop, PREVIOUS>,

    Transition<EvKeyPressX<CbDefaultKeyboardBehavior, OrKeyboard>, MsLeakyLung, ABORT>,
    Transition<EvKeyPressZ<CbDefaultKeyboardBehavior, OrKeyboard>, MsPatientObstruction, ABORT>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(40);
    configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
    configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace pc_cycle_inner_states
}  // namespace sm_respira_1
