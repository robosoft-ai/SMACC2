namespace sm_dance_bot_strikes_back
{
namespace s_pattern_states
{
// STATE DECLARATION
struct StiSPatternForward2 : public smacc::SmaccState<StiSPatternForward2, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

      Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiSPatternRotate3>,
      Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StiSPatternRotate2>

      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>();
    configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure()
  {
    auto &superstate = this->context<SS>();

    double extrasecurityMargin = 0.2;

    auto forwardBehavior = this->getOrthogonal<OrNavigation>()
                               ->getClientBehavior<CbNavigateForward>();

    cl_lidar::ClLidarSensor *lidarClient;
    this->requiresClient(lidarClient);
    auto lidarData = lidarClient->getComponent<CpLidarSensorData>();

    if (!std::isnan(lidarData->forwardObstacleDistance))
      forwardBehavior->forwardDistance = lidarData->forwardObstacleDistance - extrasecurityMargin; /*extra security margin for easy dynamic implementation of dynamic-smotion*/

    else
      forwardBehavior->forwardDistance = superstate.pitch2_lenght_meters();
  }
};
} // namespace s_pattern_states
} // namespace sm_dance_bot_strikes_back