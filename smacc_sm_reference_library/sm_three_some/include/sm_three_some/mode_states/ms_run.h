#include <smacc/smacc.h>
namespace sm_three_some
{
// STATE DECLARATION
class MsRun : public smacc::SmaccState<MsRun, SmThreeSome, StState1>
{
public:
   using SmaccState::SmaccState;
};
} // namespace sm_three_some