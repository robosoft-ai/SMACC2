namespace sm_respira_1
{
// STATE DECLARATION
struct StSystemShutdown : smacc::SmaccState<StSystemShutdown, MsShutdown>
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

    // Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, SsACCycle, TIMEOUT>,
    // Transition<smacc::EvTopicMessage<CbWatchdogSubscriberBehavior, OrSubscriber>, SsACCycle>,
    // Keyboard events
    // Transition<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>, SsACCycle, MOVE>,
    // Transition<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>, SsCMVCycle, BUILD>,
    // Transition<EvKeyPressC<CbDefaultKeyboardBehavior, OrKeyboard>, SsPCCycle, ATTACK>

    >
    reactions;

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
