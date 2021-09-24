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

namespace sm_three_some
{
// STATE DECLARATION
struct StState2 : smacc2::SmaccState<StState2, MsRun>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : ABORT
  {
  };
  struct NEXT : SUCCESS
  {
  };
  struct PREVIOUS : ABORT
  {
  };

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StState3, TIMEOUT>,
    Transition<EvAllGo<SrAllEventsGo>, StState3>,
    // Keyboard events
    Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StState1, PREVIOUS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StState3, NEXT>>
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10);
    configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
    configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();

    // Create State Reactor
    // auto sbAll = static_createStateReactor<SrAllEventsGo>();

    auto sbAll = static_createStateReactor<
      smacc2::state_reactors::SrAllEventsGo, smacc2::state_reactors::EvAllGo<SrAllEventsGo>,
      mpl::list<
        EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>,
        EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>,
        EvKeyPressC<CbDefaultKeyboardBehavior, OrKeyboard>>>();

    /*
    sbAll->addInputEvent<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>>();
    sbAll->addInputEvent<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>>();
    sbAll->addInputEvent<EvKeyPressC<CbDefaultKeyboardBehavior, OrKeyboard>>();
    sbAll->setOutputEvent<EvAllGo<SrAllEventsGo>>();
    */
  }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_three_some
