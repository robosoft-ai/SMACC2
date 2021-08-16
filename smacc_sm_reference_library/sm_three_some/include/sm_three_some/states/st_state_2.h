namespace sm_three_some
{
// STATE DECLARATION
struct StState2 : smacc::SmaccState<StState2, MsRun>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : SUCCESS
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
      smacc::state_reactors::SrAllEventsGo, smacc::state_reactors::EvAllGo<SrAllEventsGo>,
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

  void onEntry() { RCLCPP_INFO(getNode()->get_logger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getNode()->get_logger(), "On Exit!"); }
};
}  // namespace sm_three_some