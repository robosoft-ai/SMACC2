namespace sm_three_some
{
// STATE DECLARATION
struct StState4 : smacc::SmaccState<StState4, MsRun>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : SUCCESS
  {
  };
  struct NEXT : SUCCESS
  {
  };

  typedef mpl::list<Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, SS1::Ss1, TIMEOUT>>
    transitions;

  struct PREVIOUS : ABORT
  {
  };

  // STATE FUNCTIONS
  static void staticConfigure() { configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10); }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getNode()->get_logger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getNode()->get_logger(), "On Exit!"); }
};
}  // namespace sm_three_some