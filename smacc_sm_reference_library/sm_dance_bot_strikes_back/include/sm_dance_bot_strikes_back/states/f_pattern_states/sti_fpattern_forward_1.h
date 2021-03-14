
namespace sm_dance_bot_strikes_back
{
namespace f_pattern_states
{
// STATE DECLARATION
template <typename SS>
struct StiFPatternForward1 : public smacc::SmaccState<StiFPatternForward1<SS>, SS>
{
  typedef SmaccState<StiFPatternForward1<SS>, SS> TSti;
  using TSti::SmaccState;
  using TSti::context_type;

  using TSti::configure_orthogonal;

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiFPatternReturn1<SS>>

      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    TSti::template configure_orthogonal<OrNavigation, CbNavigateForward>();
    TSti::template configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure()
  {
    cl_lidar::ClLidarSensor *lidarClient;
    this->requiresClient(lidarClient);

    auto lidarData = lidarClient->getComponent<CpLidarSensorData>();

    auto forwardBehavior = TSti::template getOrthogonal<OrNavigation>()
                               ->template getClientBehavior<CbNavigateForward>();

    forwardBehavior->forwardDistance = lidarData->forwardObstacleDistance;
    RCLCPP_INFO(this->getNode()->get_logger(), "Going forward in F pattern, distance to wall: %lf", *(forwardBehavior->forwardDistance));
  }
};
}
}