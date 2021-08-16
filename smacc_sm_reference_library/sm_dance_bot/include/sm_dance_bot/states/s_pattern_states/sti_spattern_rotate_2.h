namespace sm_dance_bot
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternRotate2 : smacc::SmaccState<StiSPatternRotate2, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiSPatternForward2>,
    Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StiSPatternForward1>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    float offset = 0;
    float angle = 0;
    if (SS::direction() == TDirection::LEFT)
      angle = 90 + offset;
    else
      angle = -90 - offset;

    configure_orthogonal<OrNavigation, CbAbsoluteRotate>(angle);
    configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure()
  {
    auto & superstate = this->context<SS>();
    RCLCPP_INFO(
      getNode()->get_logger(),
      "[StiSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d",
      superstate.iteration_count, SS::total_iterations());
  }
};
}  // namespace s_pattern_states
}  // namespace sm_dance_bot
