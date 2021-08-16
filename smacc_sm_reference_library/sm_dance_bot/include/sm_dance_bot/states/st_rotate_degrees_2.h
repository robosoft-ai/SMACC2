#include <smacc/smacc.h>
namespace sm_dance_bot
{
// STATE DECLARATION
struct StRotateDegrees2 : smacc::SmaccState<StRotateDegrees2, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
 
  Transition<EvCbSuccess<CbRotate, OrNavigation>, StNavigateToWaypointsX>,
  Transition<EvCbFailure<CbRotate, OrNavigation>, StNavigateToWaypointsX>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbRotate>(/*30*/ -90);
    configure_orthogonal<OrLED, CbLEDOff>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }
};
}