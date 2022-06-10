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

namespace sm_multi_panda_sim
{
// STATE DECLARATION
struct CompletingStObserve : smacc2::SmaccState<CompletingStObserve, MsCompleting>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct completing_sequence_a : SUCCESS{};
  struct completing_sequence_b : SUCCESS{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, CompletingSequenceBLoop, SUCCESS>,
    // Transition<smacc2::EvTopicMessage<CbWatchdogSubscriberBehavior, OrSubscriber>, SsExecuteSequenceA>,
    // Keyboard events
    // Transition<EvKeyPressF<CbDefaultKeyboardBehavior, OrKeyboard>, MsStarting, SUCCESS>,
    Transition<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>, CompletingSequenceALoop, SUCCESS>,
    // Transition<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>, SsExecuteSequenceA, execute_sequence_a>,
    Transition<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>, CompletingSequenceBLoop, SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10);
    configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();
  }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_multi_panda_sim
