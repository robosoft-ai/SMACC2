#include <smacc/smacc.h>
namespace sm_dance_bot
{
// STATE DECLARATION
struct StNavigateForward2 : smacc::SmaccState<StNavigateForward2, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<

  Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StRotateDegrees5>,
  Transition<EvCbFailure<CbNavigateForward, OrNavigation>, StNavigateToWaypointsX>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateForward>(1);
    configure_orthogonal<OrLED, CbLEDOff>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }
};
} // namespace sm_dance_bot