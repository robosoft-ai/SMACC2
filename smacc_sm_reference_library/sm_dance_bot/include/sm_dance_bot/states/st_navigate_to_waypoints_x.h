#include <smacc/smacc.h>
namespace sm_dance_bot
{
// STATE DECLARATION
struct StNavigateToWaypointsX : smacc::SmaccState<StNavigateToWaypointsX, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

// CUSTOM TRANSITION TAGS
  struct TRANSITION_1 : SUCCESS{};
  struct TRANSITION_2 : SUCCESS{};
  struct TRANSITION_3 : SUCCESS{};
  struct TRANSITION_4 : SUCCESS{};
  struct TRANSITION_5 : SUCCESS{};

// TRANSITION TABLE
  typedef mpl::list<

  Transition<EvWaypoint0<ClMoveBaseZ, OrNavigation>, SS1::SsRadialPattern1, TRANSITION_1>,
  Transition<EvWaypoint1<ClMoveBaseZ, OrNavigation>, SS2::SsRadialPattern2, TRANSITION_2>,
  Transition<EvWaypoint2<ClMoveBaseZ, OrNavigation>, SS3::SsRadialPattern3, TRANSITION_3>,
  Transition<EvWaypoint3<ClMoveBaseZ, OrNavigation>, SS4::SsFPattern1, TRANSITION_4>,
  Transition<EvWaypoint4<ClMoveBaseZ, OrNavigation>, SS5::SsSPattern1, TRANSITION_5>,
  Transition<EvCbFailure<ClMoveBaseZ, OrNavigation>, StNavigateToWaypointsX>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrLED, CbLEDOn>();
    configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
    configure_orthogonal<OrNavigation, CbNavigateNextWaypoint>();
  }

  void runtimeConfigure()
  {
  }
};
} // namespace sm_dance_bot