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
struct StObserve4 : smacc2::SmaccState<StObserve4, MsMode4>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct a_sequence_4 : SUCCESS{};
  struct mode_4_sequence_c : SUCCESS{};
  struct mode_4_sequence_d : SUCCESS{};
  struct mode_4_sequence_b : SUCCESS{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>, ASequenceLoop4, SUCCESS>,
    Transition<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>, Mode4SequenceB, SUCCESS>,
    Transition<EvKeyPressD<CbDefaultKeyboardBehavior, OrKeyboard>, Mode4SequenceCLoop, SUCCESS>,
    Transition<EvKeyPressG<CbDefaultKeyboardBehavior, OrKeyboard>, Mode4SequenceDLoop, SUCCESS>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, Mode4SequenceDLoop, SUCCESS>

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
