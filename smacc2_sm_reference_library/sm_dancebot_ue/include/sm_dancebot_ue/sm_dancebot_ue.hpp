// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <smacc2/smacc.hpp>
#include <smacc2/client_behaviors/cb_sleep_for.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

// CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_ros_timer.hpp>

#include <nav2z_client/client_behaviors.hpp>
#include <nav2z_client/nav2z_client.hpp>

// #include <sm_dancebot_ue/clients/nav2z_client/client_behaviors/cb_navigate_next_waypoint.hpp>
#include <sm_dancebot_ue/clients/cl_nav2z/client_behaviors/cb_position_control_free_space.hpp>
#include <sm_dancebot_ue/clients/cl_nav2z/client_behaviors/cb_navigate_next_waypoint_free.hpp>
#include <sm_dancebot_ue/clients/cl_nav2z/client_behaviors/cb_load_waypoints_file.hpp>
#include <sm_dancebot_ue/clients/cl_nav2z/client_behaviors/cb_pure_spinning.hpp>
#include <sm_dancebot_ue/clients/cl_nav2z/client_behaviors/cb_active_stop.hpp>

#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.hpp>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.hpp>
#include <ros_publisher_client/client_behaviors/cb_publish_once.hpp>

#include <ros_publisher_client/cl_ros_publisher.hpp>

//STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.hpp>
#include <sr_conditional/sr_conditional.hpp>
#include <sr_event_countdown/sr_event_countdown.hpp>


// ORTHOGONALS
#include <sm_dancebot_ue/orthogonals/or_navigation.hpp>
#include <sm_dancebot_ue/orthogonals/or_obstacle_perception.hpp>

using namespace cl_nav2z;
using namespace smacc2::state_reactors;

namespace sm_dancebot_ue
{
//STATE FORWARD DECLARATIONS
class StAcquireSensors;
class StInitialRoadWaypointsX;
class StNavigateFieldWaypointsX;
class StBackOnRoadWaypointsX;
class StTurnAround;
class StFinalState;

//SUPERSTATE FORWARD DECLARATIONS
//MODE STATES FORWARD DECLARATIONS
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

}  // namespace sm_dancebot_ue

using namespace sm_dancebot_ue;
using namespace cl_ros_timer;
using namespace smacc2;

namespace sm_dancebot_ue
{
/// \brief Advanced example of state machine with smacc that shows multiple techniques
///  for the development of state machines
struct SmDanceBotUE : public smacc2::SmaccStateMachineBase<SmDanceBotUE, MsDanceBotRunMode>
{
  typedef mpl::bool_<false> shallow_history;
  typedef mpl::bool_<false> deep_history;
  typedef mpl::bool_<false> inherited_deep_history;

  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrNavigation>();
    this->createOrthogonal<OrObstaclePerception>();
  }
};

}  // namespace sm_dancebot_ue

//MODE STATES
#include <sm_dancebot_ue/modestates/ms_dance_bot_run_mode.hpp>

#include <sm_dancebot_ue/modestates/ms_dance_bot_recovery_mode.hpp>

//SUPERSTATES
#include <sm_dancebot_ue/superstates/ss_f_pattern_1.hpp>
#include <sm_dancebot_ue/superstates/ss_radial_pattern_1.hpp>
#include <sm_dancebot_ue/superstates/ss_radial_pattern_2.hpp>
#include <sm_dancebot_ue/superstates/ss_radial_pattern_3.hpp>
#include <sm_dancebot_ue/superstates/ss_s_pattern_1.hpp>

//STATES
#include <sm_dancebot_ue/states/st_acquire_sensors.hpp>

#include <sm_dancebot_ue/states/st_inital_road_waypoints_x.hpp>
#include <sm_dancebot_ue/states/st_navigate_field_waypoints_x.hpp>
#include <sm_dancebot_ue/states/st_back_on_road_waypoints_x.hpp>
#include <sm_dancebot_ue/states/st_turn_around.hpp>
#include <sm_dancebot_ue/states/st_final_state.hpp>
