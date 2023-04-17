#include <smacc2/smacc.hpp>

namespace sm_atomic_mode_states
{
using namespace cl_ros_timer;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
struct State1 : smacc2::SmaccState<State1, MsState1>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, MsState2, SUCCESS>

    >reactions;


// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimerCountdownLoop>(3);  // EvTimer triggers each 3 client ticks
        configure_orthogonal<OrTimer, CbTimerCountdownOnce>(2); // EvTimer triggers once at 10 client ticks
        configure_orthogonal<OrTimer, CbUpdatableTest>();

    }

    void runtimeConfigure()
    {
    }

    void onEntry()
    {
        RCLCPP_INFO(getLogger(),"On Entry!");
    }

    void onExit()
    {
        RCLCPP_INFO(getLogger(),"On Exit!");
    }

};
} // namespace sm_atomic_mode_states
