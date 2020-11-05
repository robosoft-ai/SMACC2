#include <smacc/smacc.h>
namespace sm_dance_bot
{
// STATE DECLARATION
class MsDanceBotRecoveryMode : public smacc::SmaccState<MsDanceBotRecoveryMode, SmDanceBot>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
   typedef mpl::list< 
   
   Transition<EvGlobalError, sc::deep_history<MsDanceBotRunMode::LastDeepState>> 
   
   >reactions;
   // typedef Transition<EvGlobalError, MsDanceBotRunMode> reactions;
};
}