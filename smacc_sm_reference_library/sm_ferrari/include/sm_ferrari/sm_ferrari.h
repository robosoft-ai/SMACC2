#pragma once

#include <smacc/smacc.h>

// CLIENTS
//#include <ros_timer_client/cl_ros_timer.h>
#include <keyboard_client/cl_keyboard.h>

// ORTHOGONALS
//#include "orthogonals/or_timer.h"
//#include "orthogonals/or_updatable_publisher.h"
#include "orthogonals/or_subscriber.h"
#include "orthogonals/or_keyboard.h"

//using namespace cl_ros_timer;
//using namespace cl_ros_publisher;
using namespace cl_keyboard;

using namespace sm_ferrari::cl_subscriber;

#include <keyboard_client/client_behaviors/cb_default_keyboard_behavior.h>
#include "clients/cl_subscriber/client_behaviors/cb_my_subscriber_behavior.h"

//#include <ros_timer_client/client_behaviors/cb_ros_timer.h>
//#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.h>

//STATE REACTORS
//#include <sr_all_events_go/sr_all_events_go.h>
#include <eg_conditional_generator/eg_conditional_generator.h>

using namespace smacc;
using namespace smacc::state_reactors;
using namespace smacc::default_events;
using namespace smacc::event_generators;

namespace sm_ferrari
{
//SUPERSTATES
namespace SS1
{
class Ss1;
} // namespace SS1

//SUPERSTATES
namespace SS2
{
class Ss2;
} // namespace SS1


//STATES
class StState1; // first state specially needs a forward declaration
class StState2;
class StState3;
class StState4;

class MsRun;
class MsRecover;

struct EvToDeep : sc::event<EvToDeep>
{
};

struct EvFail : sc::event<EvFail>
{
};

// STATE MACHINE
struct SmFerrari
    : public smacc::SmaccStateMachineBase<SmFerrari, MsRun>
{
    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        //this->createOrthogonal<OrTimer>();
        //this->createOrthogonal<OrUpdatablePublisher>();
        this->createOrthogonal<OrKeyboard>();
        this->createOrthogonal<OrSubscriber>();
    }
};
} // namespace sm_ferrari

// MODE STATES
#include <sm_ferrari/mode_states/ms_run.h>
#include <sm_ferrari/mode_states/ms_recover.h>

//STATES
#include <sm_ferrari/states/st_state_1.h>
#include <sm_ferrari/states/st_state_2.h>
#include <sm_ferrari/states/st_state_3.h>
#include <sm_ferrari/states/st_state_4.h>

#include <sm_ferrari/superstates/ss_superstate_1.h>
#include <sm_ferrari/superstates/ss_superstate_2.h>