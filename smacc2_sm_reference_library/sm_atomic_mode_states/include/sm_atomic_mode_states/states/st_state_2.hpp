#include <smacc2/smacc.hpp>

namespace sm_atomic_mode_states
{
// STATE DECLARATION
struct State2 : smacc2::SmaccState<State2, MsState2>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<

    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, MsState1, SUCCESS>

    >reactions;


// STATE FUNCTIONS
    static void staticConfigure()
    {
        configure_orthogonal<OrTimer, CbTimerCountdownOnce>(2); // EvTimer triggers once at 10 client ticks
        configure_orthogonal<OrTimer, CbUpdatableTest>();
    }

    void runtimeConfigure()
    {
        RCLCPP_INFO(getLogger(),"Entering State2");
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
}
