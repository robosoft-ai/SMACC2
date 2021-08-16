namespace sm_ferrari
{
using namespace std::chrono_literals;

// STATE DECLARATION
struct StState2 : smacc::SmaccState<StState2, MsRun>, smacc::ISmaccUpdatable
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

    // Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StState3, TIMEOUT>,
    // Transition<EvAllGo<SrAllEventsGo>, StState3>,
    // Keyboard events
    Transition<EvKeyPressP<CbDefaultKeyboardBehavior, OrKeyboard>, StState1, PREVIOUS>,
    Transition<EvKeyPressN<CbDefaultKeyboardBehavior, OrKeyboard>, StState3, NEXT>,
    Transition<EvMyBehavior<CbMySubscriberBehavior, OrSubscriber>, StState3, NEXT>,
    Transition<EvTrue<EgConditionalGenerator, StState2>, StState3, NEXT>

    >
    reactions;

  static int k;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    // configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10);
    // configure_orthogonal<OrSubscriber, CbWatchdogSubscriberBehavior>();
    configure_orthogonal<OrSubscriber, CbMySubscriberBehavior>();
    // configure_orthogonal<OrUpdatablePublisher, CbDefaultPublishLoop>();
    configure_orthogonal<OrKeyboard, CbDefaultKeyboardBehavior>();

    // Create State Reactor
    // auto sbAll = static_createStateReactor<SrAllEventsGo>();
    // sbAll->addInputEvent<EvKeyPressA<CbDefaultKeyboardBehavior, OrKeyboard>>();
    // sbAll->addInputEvent<EvKeyPressB<CbDefaultKeyboardBehavior, OrKeyboard>>();
    // sbAll->addInputEvent<EvKeyPressC<CbDefaultKeyboardBehavior, OrKeyboard>>();
    // sbAll->setOutputEvent<EvAllGo<SrAllEventsGo>>();

    static_createEventGenerator<EgConditionalGenerator>(ConditionalGeneratorMode::ON_UPDATE);
  }

  void runtimeConfigure()
  {
    k = 0;

    auto eg = this->getEventGenerator<EgConditionalGenerator>();
    eg->setPredicateFunction([=] { return this->eventGeneratorPredicate(this); });
  }

  bool eventGeneratorPredicate(ISmaccState * st)
  {
    auto res = k > 300;
    RCLCPP_INFO(st->getNode()->get_logger(), "[State2] checking k: %d  > 300 == %d", k, res);
    rclcpp::sleep_for(10ms);
    // rclcpp::Duration(10ms).sleep();
    return res;
  }

  virtual void update() override
  {
    k++;
    RCLCPP_INFO(getNode()->get_logger(), "[State2] internally updating k: %d", k);
  }

  void onEntry() { RCLCPP_INFO(getNode()->get_logger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getNode()->get_logger(), "On Exit!"); }
};

int StState2::k = 0;
}  // namespace sm_ferrari