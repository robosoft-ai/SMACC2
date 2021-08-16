namespace sm_dance_bot_strikes_back
{
namespace f_pattern_states
{
// STATE DECLARATION
template <typename SS>
struct StiFPatternRotate1 : smacc::SmaccState<StiFPatternRotate1<SS>, SS>
{
  typedef SmaccState<StiFPatternRotate1<SS>, SS> TSti;
  using TSti::context;
  using TSti::getOrthogonal;

  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiFPatternStartLoop<SS>>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    //TSti::template configure_orthogonal<OrNavigation, CbRotate>(angle);
    TSti::template configure_orthogonal<
      OrNavigation, CbAbsoluteRotate>();  // absolute aligned to the y-axis
    TSti::template configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure()
  {
    auto & superstate = TSti::template context<SS>();

    auto initialStateAngle = superstate.initialStateAngle;

    float angle = 0;
    double offset = 9;  // for a better behaving

    if (SS::direction() == TDirection::LEFT)
      angle = -offset;
    else
      angle = +offset;

    auto absoluteRotateBehavior =
      TSti::template getOrthogonal<OrNavigation>()->template getClientBehavior<CbAbsoluteRotate>();

    absoluteRotateBehavior->absoluteGoalAngleDegree = initialStateAngle + angle;
    RCLCPP_INFO(
      this->getNode()->get_logger(), "Fpattern, rotate to: %lf",
      *(absoluteRotateBehavior->absoluteGoalAngleDegree));
  }
};
}  // namespace f_pattern_states
}  // namespace sm_dance_bot_strikes_back