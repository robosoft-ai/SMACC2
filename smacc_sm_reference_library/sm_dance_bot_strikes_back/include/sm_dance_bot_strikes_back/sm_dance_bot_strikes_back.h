#include <smacc/smacc.h>

#include <sensor_msgs/msg/laser_scan.hpp>

// CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_ros_timer.h>

#include <multirole_sensor_client/client_behaviors/cb_default_multirole_sensor_behavior.h>

#include <move_base_z_client_plugin/move_base_z_client_plugin.h>
#include <move_base_z_client_plugin/client_behaviors.h>

using namespace cl_move_base_z;

#include <sm_dance_bot_strikes_back/clients/cl_led/client_behaviors/cb_led_on.h>
#include <sm_dance_bot_strikes_back/clients/cl_led/client_behaviors/cb_led_off.h>

#include <sm_dance_bot_strikes_back/clients/cl_lidar/client_behaviors/cb_lidar_sensor.h>
#include <sm_dance_bot_strikes_back/clients/cl_temperature_sensor/client_behaviors/cb_custom_condition_temperature_sensor.h>

#include <sm_dance_bot_strikes_back/clients/cl_string_publisher/client_behaviors/cb_string_publisher.h>
#include <sm_dance_bot_strikes_back/clients/cl_service3/client_behaviors/cb_service3.h>

#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.h>
#include <ros_publisher_client/client_behaviors/cb_publish_once.h>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.h>

#include <ros_publisher_client/cl_ros_publisher.h>

using namespace sm_dance_bot_strikes_back::cl_lidar;
using namespace sm_dance_bot_strikes_back::cl_service3;
using namespace sm_dance_bot_strikes_back::cl_string_publisher;
using namespace sm_dance_bot_strikes_back::cl_temperature_sensor;
using namespace sm_dance_bot_strikes_back::cl_led;
//using namespace sm_dance_bot_strikes_back::cl_move_base_z;
//using namespace sm_dance_bot_strikes_back::cl_updatable_publisher;

//STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.h>
#include <sr_event_countdown/sr_event_countdown.h>
#include <sr_conditional/sr_conditional.h>

using namespace smacc::state_reactors;

// ORTHOGONALS
#include <sm_dance_bot_strikes_back/orthogonals/or_navigation.h>
#include <sm_dance_bot_strikes_back/orthogonals/or_obstacle_perception.h>
#include <sm_dance_bot_strikes_back/orthogonals/or_led.h>
#include <sm_dance_bot_strikes_back/orthogonals/or_temperature_sensor.h>
#include <sm_dance_bot_strikes_back/orthogonals/or_string_publisher.h>
#include <sm_dance_bot_strikes_back/orthogonals/or_service3.h>
#include <sm_dance_bot_strikes_back/orthogonals/or_timer.h>
#include <sm_dance_bot_strikes_back/orthogonals/or_updatable_publisher.h>

namespace sm_dance_bot_strikes_back
{
//STATE FORWARD DECLARATIONS
class StAcquireSensors;
class StEventCountDown;
class StNavigateToWaypointsX;
class StFpatternPrealignment;
class StSpatternPrealignment;

//SUPERSTATE FORWARD DECLARATIONS

// MODE STATES FORWARD DECLARATIONS
class MsDanceBotRunMode;
class MsDanceBotRecoveryMode;

namespace SS1
{
class SsRadialPattern1;
}

namespace SS2
{
class SsRadialPattern2;
}

namespace SS3
{
class SsRadialPattern3;
}

namespace SS4
{
class SsFPattern1;
}

namespace SS5
{
class SsSPattern1;
}

// custom smd_dance_bot event
struct EvGlobalError : sc::event<EvGlobalError>
{
};

} // namespace sm_dance_bot_strikes_back

using namespace sm_dance_bot_strikes_back;
using namespace cl_ros_timer;
using namespace smacc;

namespace sm_dance_bot_strikes_back
{
/// \brief Advanced example of state machine with smacc that shows multiple techniques
///  for the development of state machines
struct SmDanceBotStrikesBack
    : public smacc::SmaccStateMachineBase<SmDanceBotStrikesBack, MsDanceBotRunMode>
{
    int counter_1;
    bool rt_ready_flag;

    typedef mpl::bool_<false> shallow_history;
    typedef mpl::bool_<false> deep_history;
    typedef mpl::bool_<false> inherited_deep_history;

    using SmaccStateMachineBase::SmaccStateMachineBase;

    virtual void onInitialize() override
    {
        this->setGlobalSMData("counter_1", counter_1);
        this->setGlobalSMData("rt_ready_flag", rt_ready_flag);

        this->createOrthogonal<OrNavigation>();
        this->createOrthogonal<OrObstaclePerception>();
        this->createOrthogonal<OrLED>();
        this->createOrthogonal<OrTemperatureSensor>();
        this->createOrthogonal<OrStringPublisher>();
        this->createOrthogonal<OrService3>();
        this->createOrthogonal<OrTimer>();
        this->createOrthogonal<OrUpdatablePublisher>();
    }
};

} // namespace sm_dance_bot_strikes_back

//MODE STATES
#include <sm_dance_bot_strikes_back/modestates/ms_dance_bot_run_mode.h>
#include <sm_dance_bot_strikes_back/modestates/ms_dance_bot_recovery_mode.h>

//SUPERSTATES
#include <sm_dance_bot_strikes_back/superstates/ss_radial_pattern_1.h>
#include <sm_dance_bot_strikes_back/superstates/ss_radial_pattern_2.h>
#include <sm_dance_bot_strikes_back/superstates/ss_radial_pattern_3.h>
#include <sm_dance_bot_strikes_back/superstates/ss_f_pattern_1.h>
#include <sm_dance_bot_strikes_back/superstates/ss_s_pattern_1.h>

//STATES
#include <sm_dance_bot_strikes_back/states/st_acquire_sensors.h>
#include <sm_dance_bot_strikes_back/states/st_event_count_down.h>
#include <sm_dance_bot_strikes_back/states/st_fpattern_prealignment.h>
#include <sm_dance_bot_strikes_back/states/st_spattern_prealignment.h>

#include <sm_dance_bot_strikes_back/states/st_navigate_to_waypoints_x.h>
