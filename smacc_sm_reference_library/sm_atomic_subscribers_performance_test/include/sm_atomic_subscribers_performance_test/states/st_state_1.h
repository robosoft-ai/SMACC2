#include <smacc/smacc.h>

namespace sm_atomic_subscribers_performance_test
{
using namespace smacc::default_transition_tags;

// STATE DECLARATION
struct State1 : smacc::SmaccState<State1, SmAtomicSubscribersPerformanceTest>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvTopicMessage<SmaccSubscriberClient<std_msgs::msg::Int16>, OrSubscriber>, State2> >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}

  void onEntry() {}

  void onExit() {}
};
}  // namespace sm_atomic_subscribers_performance_test
