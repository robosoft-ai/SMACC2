#include <smacc/smacc.h>

using namespace boost;
using namespace smacc;

namespace sm_atomic_performance_test
{
//STATE
class State1;
class State2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmAtomicPerformanceTest
: public smacc::SmaccStateMachineBase<SmAtomicPerformanceTest, State1>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override {}
};

}  // namespace sm_atomic_performance_test

#include "states/st_state_1.h"
#include "states/st_state_2.h"
