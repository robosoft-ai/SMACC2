// Copyright 2026 RobosoftAI Inc.
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

#pragma once

#include <smacc2/smacc.hpp>

// CLIENTS
#include <cl_generic_sensor/cl_generic_sensor.hpp>
#include <cl_px4_mr/cl_px4_mr.hpp>

// COMPONENTS (beyond the ClPx4Mr defaults)
#include <cl_px4_mr/components/cp_forward_obstacle_guard.hpp>
#include <cl_px4_mr/components/cp_tf_broadcaster.hpp>

// CLIENT BEHAVIORS
#include <cl_px4_mr/client_behaviors/cb_arm_px4.hpp>
#include <cl_px4_mr/client_behaviors/cb_ascend_to_altitude.hpp>
#include <cl_px4_mr/client_behaviors/cb_connect_micro_ros_agent.hpp>
#include <cl_px4_mr/client_behaviors/cb_disarm_px4.hpp>
#include <cl_px4_mr/client_behaviors/cb_follow_ned_path.hpp>
#include <cl_px4_mr/client_behaviors/cb_go_to_location.hpp>
#include <cl_px4_mr/client_behaviors/cb_hold_position.hpp>
#include <cl_px4_mr/client_behaviors/cb_land.hpp>
#include <cl_px4_mr/client_behaviors/cb_obstacle_guard.hpp>
#include <cl_px4_mr/client_behaviors/cb_orbit_location.hpp>
#include <cl_px4_mr/client_behaviors/cb_spiral_up.hpp>
#include <cl_px4_mr/client_behaviors/cb_takeoff.hpp>
#include <cl_px4_mr/client_behaviors/cb_wait_for_heading_stable.hpp>
#include <cl_px4_mr/client_behaviors/cb_yaw_rotate.hpp>
#include <smacc2/client_behaviors/cb_sleep_for.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

// CONFIG (mission constants and the helpers derived from them)
#include <config/mission_constants.hpp>

using namespace boost;
using namespace smacc2;

namespace sm_cl_px4_mr_test_5
{

// the LiDAR client: a generic PointCloud2 sensor with a message watchdog
using ClLidar = cl_generic_sensor::ClGenericSensor<sensor_msgs::msg::PointCloud2>;

// posted by StObstacleHold when the vehicle has been stopped too often
struct EvLandHere : sc::event<EvLandHere>
{
};

}  // namespace sm_cl_px4_mr_test_5

// ORTHOGONALS
#include <sm_cl_px4_mr_test_5/orthogonals/or_lidar.hpp>
#include <sm_cl_px4_mr_test_5/orthogonals/or_px4.hpp>

namespace sm_cl_px4_mr_test_5
{

// MODE STATES (forward declarations)
class MsDisarmedOnGround;
class MsArmedOnGround;
class MsTakeoff;
class MsInFlight;
class MsLanding;
class MsLanded;

// SUPERSTATES (forward declarations)
class SsCaveMission;

// STATES (forward declarations), in mission order
class StPause;
class StConnectMicroROSAgent;
class StWaitForReady;
class StArmPX4;
class StPrepareForTakeoff;
class StTakeoff;
class StAscend;
class StFlyOutbound;
class StFlyDeeper;
class StHoldAtTurnaround;
class StTurnAround;
class StFlyInbound;
class StFlyToBaseStation;
class StOrbitBaseStation;
class StSpiralUpBaseStation;
class StOrbitAtTop;
class StReturnToPad;
class StPreLandDescent;
class StObstacleHold;
class StReturnHome;
class StLand;
class StLanded;

// STATE MACHINE
struct SmClPx4MrTest5 : public smacc2::SmaccStateMachineBase<SmClPx4MrTest5, MsDisarmedOnGround>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrPx4>();
    this->createOrthogonal<OrLidar>();
  }
};

}  // namespace sm_cl_px4_mr_test_5

// MODE STATE INCLUDES (after the SM definition, before regular states)
#include <sm_cl_px4_mr_test_5/modestates/ms_disarmed_on_ground.hpp>
#include <sm_cl_px4_mr_test_5/modestates/ms_armed_on_ground.hpp>
#include <sm_cl_px4_mr_test_5/modestates/ms_takeoff.hpp>
#include <sm_cl_px4_mr_test_5/modestates/ms_in_flight.hpp>
#include <sm_cl_px4_mr_test_5/modestates/ms_landing.hpp>
#include <sm_cl_px4_mr_test_5/modestates/ms_landed.hpp>

// SUPERSTATE INCLUDES (the nominal flight legs live inside it)
#include <sm_cl_px4_mr_test_5/superstates/ss_cave_mission.hpp>

// REGULAR STATE INCLUDES, in mission order
#include <sm_cl_px4_mr_test_5/states/st_pause.hpp>
#include <sm_cl_px4_mr_test_5/states/st_connect_micro_ros_agent.hpp>
#include <sm_cl_px4_mr_test_5/states/st_wait_for_ready.hpp>
#include <sm_cl_px4_mr_test_5/states/st_arm_px4.hpp>
#include <sm_cl_px4_mr_test_5/states/st_prepare_for_takeoff.hpp>
#include <sm_cl_px4_mr_test_5/states/st_takeoff.hpp>
#include <sm_cl_px4_mr_test_5/states/st_ascend.hpp>
#include <sm_cl_px4_mr_test_5/states/st_fly_outbound.hpp>
#include <sm_cl_px4_mr_test_5/states/st_fly_deeper.hpp>
#include <sm_cl_px4_mr_test_5/states/st_hold_at_turnaround.hpp>
#include <sm_cl_px4_mr_test_5/states/st_turn_around.hpp>
#include <sm_cl_px4_mr_test_5/states/st_fly_inbound.hpp>
#include <sm_cl_px4_mr_test_5/states/st_fly_to_base_station.hpp>
#include <sm_cl_px4_mr_test_5/states/st_spiral_up_base_station.hpp>
#include <sm_cl_px4_mr_test_5/states/st_orbit_at_top.hpp>
#include <sm_cl_px4_mr_test_5/states/st_orbit_base_station.hpp>
#include <sm_cl_px4_mr_test_5/states/st_return_to_pad.hpp>
#include <sm_cl_px4_mr_test_5/states/st_pre_land_descent.hpp>
#include <sm_cl_px4_mr_test_5/states/st_obstacle_hold.hpp>
#include <sm_cl_px4_mr_test_5/states/st_return_home.hpp>
#include <sm_cl_px4_mr_test_5/states/st_land.hpp>
#include <sm_cl_px4_mr_test_5/states/st_landed.hpp>
