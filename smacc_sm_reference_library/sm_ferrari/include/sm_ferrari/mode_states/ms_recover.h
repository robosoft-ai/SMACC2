#include <smacc/smacc.h>
namespace sm_ferrari
{
// STATE DECLARATION
class MsRecover : public smacc::SmaccState<MsRecover, SmFerrari>
{
public:
   using SmaccState::SmaccState;

// TRANSITION TABLE
   typedef mpl::list<
    
   Transition<EvToDeep, sc::deep_history<MsRun::LastDeepState>, SUCCESS>
   
   >reactions;
};
} // namespace sm_ferrari