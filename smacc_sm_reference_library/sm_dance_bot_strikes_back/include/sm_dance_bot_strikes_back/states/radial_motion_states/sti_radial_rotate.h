namespace sm_dance_bot_strikes_back
{
namespace radial_motion_states
{
// STATE DECLARATION
struct StiRadialRotate : smacc::SmaccState<StiRadialRotate, SS>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, StiRadialEndPoint, SUCCESS>,
  Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StiRadialLoopStart, ABORT>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbAbsoluteRotate>();
    configure_orthogonal<OrLED, CbLEDOff>();
  }

  void runtimeConfigure()
  {
    auto cbAbsRotate = this->getOrthogonal<OrNavigation>()
                           ->getClientBehavior<CbAbsoluteRotate>();

    auto &superstate = this->context<SS>();
    cbAbsRotate->absoluteGoalAngleDegree = superstate.iteration_count * SS::ray_angle_increment_degree();
  }
};
} // namespace radial_motion_states
} // namespace sm_dance_bot_strikes_back