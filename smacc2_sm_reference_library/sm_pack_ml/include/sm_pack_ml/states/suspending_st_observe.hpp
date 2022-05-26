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

namespace sm_pack_ml
{
// STATE DECLARATION
struct SuspendingStObserve : smacc2::SmaccState<SuspendingStObserve, MsSuspending>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct suspending_sequence_a : SUCCESS{};
  struct suspending_sequence_c : SUCCESS{};
  struct suspending_sequence_d : SUCCESS{};
  struct suspending_sequence_b : SUCCESS{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>, SuspendingSequenceALoop, SUCCESS>,
    Transition<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>, SuspendingSequenceB, SUCCESS>,
    Transition<EvKeyPressD<CbDefaultKeyboardBehavior, OrKeyboard>, SuspendingSequenceCLoop, SUCCESS>,
    Transition<EvKeyPressG<CbDefaultKeyboardBehavior, OrKeyboard>, SuspendingSequenceDLoop, SUCCESS>,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, SuspendingSequenceDLoop, SUCCESS>

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
}  // namespace sm_pack_ml
