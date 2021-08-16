namespace sm_dance_bot_strikes_back
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
    configure_orthogonal<OrNavigation, CbAbsoluteRotate>();
    configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure()
  {
    auto & superstate = this->context<SS>();
    RCLCPP_INFO(
      getNode()->get_logger(),
      "[StiSPatternRotate] SpatternRotate rotate: SS current iteration: %d/%d",
      superstate.iteration_count, SS::total_iterations());

    float offset = 0;
    auto absoluteRotateBehavior =
      this->getOrthogonal<OrNavigation>()->template getClientBehavior<CbAbsoluteRotate>();

    if (superstate.direction() == TDirection::RIGHT)
    {
      absoluteRotateBehavior->absoluteGoalAngleDegree = superstate.initialStateAngle - 90 - offset;
    }
    else
    {
      absoluteRotateBehavior->absoluteGoalAngleDegree = superstate.initialStateAngle + 90 + offset;
    }
  }
};
}  // namespace s_pattern_states
}  // namespace sm_dance_bot_strikes_back
