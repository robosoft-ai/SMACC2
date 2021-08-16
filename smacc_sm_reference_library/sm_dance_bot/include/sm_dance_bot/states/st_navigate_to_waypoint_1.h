#include <smacc/smacc.h>
namespace sm_dance_bot
{
// STATE DECLARATION
struct StNavigateToWaypoint1 : smacc::SmaccState<StNavigateToWaypoint1, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
  
  Transition<EvCbSuccess<CbNavigateGlobalPosition, OrNavigation>, StNavigateToWaypointsX>,
  Transition<EvCbFailure<CbNavigateGlobalPosition, OrNavigation>, StNavigateToWaypointsX>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbNavigateGlobalPosition>(0, 0, 0);
    configure_orthogonal<OrLED, CbLEDOn>();
    configure_orthogonal<OrStringPublisher, CbStringPublisher>("All Done!");
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }
};
}