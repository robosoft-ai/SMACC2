#include <smacc/smacc.h>
namespace sm_dance_bot
{
// STATE DECLARATION
struct StNavigateReverse1 : smacc::SmaccState<StNavigateReverse1, MsDanceBotRunMode>
{
   using SmaccState::SmaccState;

// TRANSITION TABLE
   typedef mpl::list<

   Transition<EvCbSuccess<CbNavigateBackwards, OrNavigation>, StRotateDegrees3>,
   Transition<EvCbFailure<CbNavigateBackwards, OrNavigation>, StNavigateToWaypointsX>
   
   >reactions;

// STATE FUNCTIONS
   static void staticConfigure()
   {
      configure_orthogonal<OrNavigation, CbNavigateBackwards>(2);
      configure_orthogonal<OrLED, CbLEDOff>();
      configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
   }
};
}