#include <smacc/smacc.h>
namespace sm_ferrari
{
// STATE DECLARATION
class MsRun : public smacc::SmaccState<MsRun, SmFerrari, StState1>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_ferrari