namespace sm_dance_bot
{
namespace radial_motion_states
{
// STATE DECLARATION
struct StiRadialEndPoint : smacc::SmaccState<StiRadialEndPoint, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiRadialReturn, SUCCESS>,
    Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiRadialRotate, ABORT>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    //RCLCPP_INFO(getNode()->get_logger(),"ssr radial end point, distance in meters: %lf", SS::ray_length_meters());
    configure_orthogonal<OrNavigation, CbNavigateForward>(SS::ray_length_meters());
    configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure() {}
};
}  // namespace radial_motion_states
}  // namespace sm_dance_bot