#include <smacc/smacc.h>
namespace sm_respira_1
{
// STATE DECLARATION
class MsLeakyLung : public smacc::SmaccState<MsLeakyLung, SmRespira1, StLeakyLungStep1>
{
public:
  using SmaccState::SmaccState;
};
}  // namespace sm_respira_1
