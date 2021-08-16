#include <smacc/smacc.h>
namespace sm_dance_bot_strikes_back
{
// STATE DECLARATION
struct StFpatternPrealignment : smacc::SmaccState<StFpatternPrealignment, MsDanceBotRunMode>
{
  using SmaccState::SmaccState;

// TRANSITION TABLE
  typedef mpl::list<
    
  Transition<EvCbSuccess<CbAbsoluteRotate, OrNavigation>, SS4::SsFPattern1>,
  Transition<EvCbFailure<CbAbsoluteRotate, OrNavigation>, StNavigateToWaypointsX>
  
  >reactions;

// STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbAbsoluteRotate>(0);
    configure_orthogonal<OrLED, CbLEDOff>();
  }
  
  void runtimeConfigure()
  { 
  }
};
}