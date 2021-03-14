namespace sm_dance_bot_strikes_back
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
  Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiRadialLoopStart, ABORT>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>();
    configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure()
  {
    cl_lidar::ClLidarSensor *lidarClient;
    this->requiresClient(lidarClient);

    auto lidarData = lidarClient->getComponent<CpLidarSensorData>();

    auto forwardBehavior = this->getOrthogonal<OrNavigation>()
                               ->getClientBehavior<CbNavigateForward>();

    forwardBehavior->forwardDistance = lidarData->forwardObstacleDistance;
  }
};
} // namespace radial_motion_states
} // namespace sm_dance_bot_strikes_back