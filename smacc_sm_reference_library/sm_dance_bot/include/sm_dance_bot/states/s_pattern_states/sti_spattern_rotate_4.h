namespace sm_dance_bot
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternRotate4 : smacc::SmaccState<StiSPatternRotate4, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiSPatternForward4>,
    Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StiSPatternForward3>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure()
  {
    auto & superstate = this->context<SS>();
    RCLCPP_INFO(
      getNode()->get_logger(),
      "[SsrSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d",
      superstate.iteration_count, SS::total_iterations());

    float offset = 0;
    float angle = 0;
    if (superstate.direction() == TDirection::LEFT)
      angle = -90 - offset;
    else
      angle = 90 + offset;

    this->configure<OrNavigation, CbAbsoluteRotate>(angle);
    this->configure<OrLED, CbLEDOff>();
  }
};
}  // namespace s_pattern_states
}  // namespace sm_dance_bot
