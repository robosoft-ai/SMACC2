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

#include <sensor_msgs/msg/laser_scan.hpp>

// CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_ros_timer.hpp>

#include <multirole_sensor_client/client_behaviors/cb_default_multirole_sensor_behavior.hpp>

#include <nav2z_client/client_behaviors.hpp>
#include <nav2z_client/nav2z_client.hpp>

using namespace cl_nav2z;

#include <ros_publisher_client/client_behaviors/cb_default_publish_loop.hpp>
#include <ros_publisher_client/client_behaviors/cb_muted_behavior.hpp>
#include <ros_publisher_client/client_behaviors/cb_publish_once.hpp>

#include <ros_publisher_client/cl_ros_publisher.hpp>


//STATE REACTORS
#include <sr_all_events_go/sr_all_events_go.hpp>
#include <sr_conditional/sr_conditional.hpp>
#include <sr_event_countdown/sr_event_countdown.hpp>

using namespace smacc2::state_reactors;

// ORTHOGONALS
#include <sm_dance_bot_coverage/orthogonals/or_navigation.hpp>
#include <sm_dance_bot_coverage/orthogonals/or_obstacle_perception.hpp>
#include <sm_dance_bot_coverage/orthogonals/or_timer.hpp>

namespace sm_dance_bot_coverage
{
//STATE FORWARD DECLARATIONS
class StAcquireSensors;
class StEventCountDown;

class StNavigateCoverage;

//SUPERSTATE FORWARD DECLARATIONS
//MODE STATES FORWARD DECLARATIONS
class MsDanceBotRunMode;
class MsDanceBotRecoveryMode;


// custom smd_dance_bot event
struct EvGlobalError : sc::event<EvGlobalError>
{
};

}  // namespace sm_dance_bot_coverage

using namespace sm_dance_bot_coverage;
using namespace cl_ros_timer;
using namespace smacc2;

namespace sm_dance_bot_coverage
{
/// \brief Advanced example of state machine with smacc that shows multiple techniques
///  for the development of state machines
struct SmDanceBotCoverage : public smacc2::SmaccStateMachineBase<SmDanceBotCoverage, MsDanceBotRunMode>
{
  int counter_1;
  bool rt_ready_flag;

  typedef mpl::bool_<false> shallow_history;
  typedef mpl::bool_<false> deep_history;
  typedef mpl::bool_<false> inherited_deep_history;

  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->setGlobalSMData("counter_1", counter_1);
    this->setGlobalSMData("rt_ready_flag", rt_ready_flag);

    this->createOrthogonal<OrNavigation>();
    this->createOrthogonal<OrObstaclePerception>();
    this->createOrthogonal<OrTimer>();
  }
};

}  // namespace sm_dance_bot_coverage

//MODE STATES
#include <sm_dance_bot_coverage/modestates/ms_dance_bot_run_mode.hpp>

#include <sm_dance_bot_coverage/modestates/ms_dance_bot_recovery_mode.hpp>

//STATES
#include <sm_dance_bot_coverage/states/st_acquire_sensors.hpp>
#include <sm_dance_bot_coverage/states/st_event_count_down.hpp>

#include <sm_dance_bot_coverage/states/st_navigate_coverage.hpp>
