#include <smacc/smacc.h>

namespace sm_atomic_performance_test
{
using namespace smacc::default_transition_tags;

// STATE DECLARATION
struct State1 : smacc::SmaccState<State1, SmAtomicPerformanceTest>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
          Transition<EvStateRequestFinish<State1>, State2>
      >
      reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
  }

  void runtimeConfigure()
  {
  }

  void onEntry()
  {
    this->postEvent<EvStateRequestFinish<State1>>();
  }

  void onExit()
  {
  }
};
}  // namespace sm_atomic_performance_test