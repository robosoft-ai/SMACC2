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
struct StObserve3 : smacc2::SmaccState<StObserve3, MsMode3>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct a_sequence_3 : SUCCESS{};
  struct b_sequence_3 : SUCCESS{};

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, BSequenceLoop3, SUCCESS>,
    // Transition<smacc2::EvTopicMessage<CbWatchdogSubscriberBehavior, OrSubscriber>, SsASequence1>,
    // Keyboard events
    // Transition<EvKeyPressF<CbDefaultKeyboardBehavior, OrKeyboard>, MsMode2, SUCCESS>,
    Transition<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>, ASequenceLoop3, SUCCESS>,
    // Transition<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>, SsASequence1, a_sequence_1>,
    Transition<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>, BSequenceLoop3, SUCCESS>

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
