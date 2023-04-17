#include <smacc2/smacc.hpp>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.hpp>

// ORTHOGONALS
#include <sm_atomic_mode_states/orthogonals/or_timer.hpp>

//CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_timer_countdown_loop.hpp>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp>
#include <sm_atomic_mode_states/client_behaviors/cb_updatable_test.hpp>

using namespace boost;
using namespace smacc2;

namespace sm_atomic_mode_states
{

//STATE
class State1;
class State2;
class MsState1;
class MsState2;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmAtomicModeStates
    : public smacc2::SmaccStateMachineBase<SmAtomicModeStates, MsState1>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->createOrthogonal<OrTimer>();
    }
};

} // namespace sm_atomic_mode_states

#include <sm_atomic_mode_states/states/ms_state_1.hpp>
#include <sm_atomic_mode_states/states/ms_state_2.hpp>

#include <sm_atomic_mode_states/states/st_state_1.hpp>
#include <sm_atomic_mode_states/states/st_state_2.hpp>
