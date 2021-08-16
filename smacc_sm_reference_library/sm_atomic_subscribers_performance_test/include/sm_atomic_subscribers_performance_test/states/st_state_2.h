#include <smacc/smacc.h>

namespace sm_atomic_subscribers_performance_test
{
// STATE DECLARATION
struct State2 : smacc::SmaccState<State2, SmAtomicSubscribersPerformanceTest>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvTopicMessage<SmaccSubscriberClient<std_msgs::msg::Int16>, OrSubscriber>, State1> >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}

  void onEntry() {}

  void onExit() {}
};
}  // namespace sm_atomic_subscribers_performance_test