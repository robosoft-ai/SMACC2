#include <smacc/smacc.h>
namespace sm_respira_1
{
// STATE DECLARATION
class MsShutdown : public smacc::SmaccState<MsShutdown, SmRespira1, StSystemShutdown>
{
public:
  using SmaccState::SmaccState;
};
}  // namespace sm_respira_1