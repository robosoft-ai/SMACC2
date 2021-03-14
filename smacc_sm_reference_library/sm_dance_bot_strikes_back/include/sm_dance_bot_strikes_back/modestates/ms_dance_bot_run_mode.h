#include <smacc/smacc.h>
namespace sm_dance_bot_strikes_back
{
// STATE DECLARATION
class MsDanceBotRunMode : public smacc::SmaccState<MsDanceBotRunMode, SmDanceBotStrikesBack, StAcquireSensors>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
   typedef mpl::list<
   
   Transition<EvGlobalError, MsDanceBotRecoveryMode> 
   
   >reactions;
};
}