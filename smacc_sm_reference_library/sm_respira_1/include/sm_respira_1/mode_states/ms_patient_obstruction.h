#include <smacc/smacc.h>
namespace sm_respira_1
{
// STATE DECLARATION
class MsPatientObstruction
: public smacc::SmaccState<MsPatientObstruction, SmRespira1, StPatientObstructionStep1>
{
public:
  using SmaccState::SmaccState;
};
}  // namespace sm_respira_1
