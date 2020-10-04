#include <smacc/smacc.h>

namespace sm_atomic
{
using namespace cl_ros_timer;
using namespace smacc::default_transition_tags;

// STATE DECLARATION
struct State1 : smacc::SmaccState<State1, SmAtomic>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<
    
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, State2, SUCCESS>
    
    >reactions;

    
// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimerCountdownLoop>(3);  // EvTimer triggers each 3 client ticks
        configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5); // EvTimer triggers once at 10 client ticks
    }

    void runtimeConfigure()
    {
    }

    void onEntry()
    {
        RCLCPP_INFO(getNode()->get_logger(), "On Entry!");
    }

    void onExit()
    {
        RCLCPP_INFO(getNode()->get_logger(), "On Exit!");
    }
  
};
} // namespace sm_atomic