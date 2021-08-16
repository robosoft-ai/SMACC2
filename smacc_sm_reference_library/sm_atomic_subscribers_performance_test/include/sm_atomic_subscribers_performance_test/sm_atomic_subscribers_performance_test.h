#include <smacc/smacc.h>
#include "orthogonals/or_subscriber.h"

using namespace boost;
using namespace smacc;

namespace sm_atomic_subscribers_performance_test
{
//STATE
class State1;
class State2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmAtomicSubscribersPerformanceTest
: public smacc::SmaccStateMachineBase<SmAtomicSubscribersPerformanceTest, State1>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override { this->createOrthogonal<OrSubscriber>(); }
};

}  // namespace sm_atomic_subscribers_performance_test

#include "states/st_state_1.h"
#include "states/st_state_2.h"
