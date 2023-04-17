#include <smacc2/smacc.hpp>

namespace sm_atomic_mode_states
{
using namespace cl_ros_timer;
using namespace smacc2::default_transition_tags;

// STATE DECLARATION
struct MsState2 : smacc2::SmaccState<MsState2, SmAtomicModeStates, State2>
{
    using SmaccState::SmaccState;

// TRANSITION TABLE
    typedef mpl::list<


    >reactions;


// STATE FUNCTIONS
    static void staticConfigure()
    {
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
