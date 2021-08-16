#include <smacc/smacc.h>

namespace sm_atomic
{
// STATE DECLARATION
struct State2 : smacc::SmaccState<State2, SmAtomic>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, State1, SUCCESS>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(
      5);  // EvTimer triggers once at 10 client ticks
  }

  void runtimeConfigure() { RCLCPP_INFO(getNode()->get_logger(), "Entering State2"); }

  void onEntry() { RCLCPP_INFO(getNode()->get_logger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getNode()->get_logger(), "On Exit!"); }
};
}  // namespace sm_atomic