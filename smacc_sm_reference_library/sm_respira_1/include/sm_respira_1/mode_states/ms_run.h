#include <smacc/smacc.h>
namespace sm_respira_1
{
// STATE DECLARATION
class MsRun : public smacc::SmaccState<MsRun, SmRespira1, StObserve>
{
public:
  using SmaccState::SmaccState;
};
}  // namespace sm_respira_1
