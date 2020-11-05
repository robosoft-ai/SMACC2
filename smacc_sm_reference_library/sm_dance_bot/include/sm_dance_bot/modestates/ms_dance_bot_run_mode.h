#include <smacc/smacc.h>
namespace sm_dance_bot
{
// STATE DECLARATION
class MsDanceBotRunMode : public smacc::SmaccState<MsDanceBotRunMode, SmDanceBot, StAcquireSensors>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
   typedef mpl::list<
   
   Transition<EvGlobalError, MsDanceBotRecoveryMode> 
   
   >reactions;
};
}