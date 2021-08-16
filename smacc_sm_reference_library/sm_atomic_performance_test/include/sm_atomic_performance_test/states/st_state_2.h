#include <smacc/smacc.h>

namespace sm_atomic_performance_test
{
// STATE DECLARATION
struct State2 : smacc::SmaccState<State2, SmAtomicPerformanceTest>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<Transition<EvStateRequestFinish<State2>, State1>> reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}

  void onEntry() { this->postEvent<EvStateRequestFinish<State2>>(); }

  void onExit() {}
};
}  // namespace sm_atomic_performance_test
