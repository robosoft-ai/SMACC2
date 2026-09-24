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
#include <cl_px4_mr/cl_px4_mr.hpp>

// CLIENT BEHAVIORS
#include <cl_px4_mr/client_behaviors/cb_arm_px4.hpp>
#include <cl_px4_mr/client_behaviors/cb_ascend_to_altitude.hpp>
#include <cl_px4_mr/client_behaviors/cb_connect_micro_ros_agent.hpp>
#include <cl_px4_mr/client_behaviors/cb_disarm_px4.hpp>
#include <cl_px4_mr/client_behaviors/cb_figure_eight.hpp>
#include <cl_px4_mr/client_behaviors/cb_grid_pattern.hpp>
#include <cl_px4_mr/client_behaviors/cb_lawnmower.hpp>
#include <cl_px4_mr/client_behaviors/cb_go_to_location.hpp>
#include <cl_px4_mr/client_behaviors/cb_land.hpp>
#include <cl_px4_mr/client_behaviors/cb_loiter.hpp>
#include <cl_px4_mr/client_behaviors/cb_sine_wave_horizontal.hpp>
#include <cl_px4_mr/client_behaviors/cb_sine_wave_vertical.hpp>
#include <cl_px4_mr/client_behaviors/cb_spiral.hpp>
#include <cl_px4_mr/client_behaviors/cb_square_spiral.hpp>
#include <cl_px4_mr/client_behaviors/cb_takeoff.hpp>
#include <cl_px4_mr/client_behaviors/cb_vs_search.hpp>
#include <smacc2/client_behaviors/cb_sleep_for.hpp>

// ORTHOGONALS
#include <sm_cl_px4_mr_test_4/orthogonals/or_px4.hpp>

// CONFIG (mission constants and the helpers derived from them)
#include <config/mission_constants.hpp>

using namespace boost;
using namespace smacc2;

namespace sm_cl_px4_mr_test_4
{

// MODE STATES (forward declarations)
class MsDisarmedOnGround;
class MsArmedOnGround;
class MsTakeoff;
class MsInFlight;
class MsLanding;
class MsLanded;

// SUPERSTATES (forward declarations)
class SsSquareSpiral1;
class SsSquareSpiral2;
class SsLawnmower1;
class SsLawnmower2;
class SsGridPattern1;
class SsVSSearch1;
class SsVSSearch2;
class SsVSSearch3;
class SsVSChain1;
class SsVSChain2;
class SsVSChain3;

// INNER STATES (forward declarations)
class StiSquareSpiral1Run;
class StiSquareSpiral2Run;
class StiLawnmower1Run;
class StiLawnmower2Run;
class StiGridPattern1Run;
class StiVSSearch1Run;
class StiVSSearch2Run;
class StiVSSearch3Run;
class StiVSChain1Run;
class StiVSChain2Run;
class StiVSChain3Run;

// STATES (forward declarations), in mission order
class StPause;
class StConnectMicroROSAgent;
class StWaitForReady;
class StArmPX4;
class StPrepareForTakeoff;
class StTakeoff;
class StAscend;
class StSpiralOffIsland;
class StTransitToSquareSpiral1;
class StTransitToSquareSpiral2;
class StTransitToLawnmower1;
class StTransitToLawnmower2;
class StTransitToGridPattern1;
class StTransitToVSSearch;
class StTransitToVSChain1;
class StTransitToVSChain2;
class StTransitToVSChain3;
class StGoToSquareCentre;
class StGoToSouthWaypoint;
class StGoToFigureEight;
class StFigureEight1;
class StFigureEight2;
class StLoiterCentroid;
class StGoToLandingZone;
class StPreLandDescent;
class StReturnHome;
class StLand;
class StLanded;

// STATE MACHINE
struct SmClPx4MrTest4 : public smacc2::SmaccStateMachineBase<SmClPx4MrTest4, MsDisarmedOnGround>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrPx4>();
  }
};

}  // namespace sm_cl_px4_mr_test_4

// MODE STATE INCLUDES (after the SM definition, before regular states)
#include <sm_cl_px4_mr_test_4/modestates/ms_disarmed_on_ground.hpp>
#include <sm_cl_px4_mr_test_4/modestates/ms_armed_on_ground.hpp>
#include <sm_cl_px4_mr_test_4/modestates/ms_takeoff.hpp>
#include <sm_cl_px4_mr_test_4/modestates/ms_in_flight.hpp>
#include <sm_cl_px4_mr_test_4/modestates/ms_landing.hpp>
#include <sm_cl_px4_mr_test_4/modestates/ms_landed.hpp>

// SUPERSTATE INCLUDES (after mode states, before leaf states: the transits
// call their neighbours' entry() / exit(), which need the complete type)
#include <sm_cl_px4_mr_test_4/superstates/ss_square_spiral_1.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_square_spiral_2.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_lawnmower_1.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_lawnmower_2.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_grid_pattern_1.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_vs_search_1.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_vs_search_2.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_vs_search_3.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_vs_chain_1.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_vs_chain_2.hpp>
#include <sm_cl_px4_mr_test_4/superstates/ss_vs_chain_3.hpp>

// REGULAR STATE INCLUDES, in mission order. One exception: st_figure_eight_1
// comes before st_go_to_figure_eight, whose target is StFigureEight1::entry().
#include <sm_cl_px4_mr_test_4/states/st_pause.hpp>
#include <sm_cl_px4_mr_test_4/states/st_connect_micro_ros_agent.hpp>
#include <sm_cl_px4_mr_test_4/states/st_wait_for_ready.hpp>
#include <sm_cl_px4_mr_test_4/states/st_arm_px4.hpp>
#include <sm_cl_px4_mr_test_4/states/st_prepare_for_takeoff.hpp>
#include <sm_cl_px4_mr_test_4/states/st_takeoff.hpp>
#include <sm_cl_px4_mr_test_4/states/st_ascend.hpp>
#include <sm_cl_px4_mr_test_4/states/st_spiral_off_island.hpp>
#include <sm_cl_px4_mr_test_4/states/st_transit_to_square_spiral_1.hpp>
#include <sm_cl_px4_mr_test_4/states/st_transit_to_square_spiral_2.hpp>
#include <sm_cl_px4_mr_test_4/states/st_transit_to_lawnmower_1.hpp>
#include <sm_cl_px4_mr_test_4/states/st_transit_to_lawnmower_2.hpp>
#include <sm_cl_px4_mr_test_4/states/st_transit_to_grid_pattern_1.hpp>
#include <sm_cl_px4_mr_test_4/states/st_transit_to_vs_search.hpp>
#include <sm_cl_px4_mr_test_4/states/st_transit_to_vs_chain_1.hpp>
#include <sm_cl_px4_mr_test_4/states/st_transit_to_vs_chain_2.hpp>
#include <sm_cl_px4_mr_test_4/states/st_transit_to_vs_chain_3.hpp>
#include <sm_cl_px4_mr_test_4/states/st_go_to_square_centre.hpp>
#include <sm_cl_px4_mr_test_4/states/st_go_to_south_waypoint.hpp>
#include <sm_cl_px4_mr_test_4/states/st_figure_eight_1.hpp>
#include <sm_cl_px4_mr_test_4/states/st_go_to_figure_eight.hpp>
#include <sm_cl_px4_mr_test_4/states/st_figure_eight_2.hpp>
#include <sm_cl_px4_mr_test_4/states/st_loiter_centroid.hpp>
#include <sm_cl_px4_mr_test_4/states/st_go_to_landing_zone.hpp>
#include <sm_cl_px4_mr_test_4/states/st_pre_land_descent.hpp>
#include <sm_cl_px4_mr_test_4/states/st_return_home.hpp>
#include <sm_cl_px4_mr_test_4/states/st_land.hpp>
#include <sm_cl_px4_mr_test_4/states/st_landed.hpp>

// INNER STATE INCLUDES (after their superstates and the states they lead to)
#include <sm_cl_px4_mr_test_4/states/square_spiral_1_inner_states/sti_square_spiral_1_run.hpp>
#include <sm_cl_px4_mr_test_4/states/square_spiral_2_inner_states/sti_square_spiral_2_run.hpp>
#include <sm_cl_px4_mr_test_4/states/lawnmower_1_inner_states/sti_lawnmower_1_run.hpp>
#include <sm_cl_px4_mr_test_4/states/lawnmower_2_inner_states/sti_lawnmower_2_run.hpp>
#include <sm_cl_px4_mr_test_4/states/grid_pattern_1_inner_states/sti_grid_pattern_1_run.hpp>
#include <sm_cl_px4_mr_test_4/states/vs_search_1_inner_states/sti_vs_search_1_run.hpp>
#include <sm_cl_px4_mr_test_4/states/vs_search_2_inner_states/sti_vs_search_2_run.hpp>
#include <sm_cl_px4_mr_test_4/states/vs_search_3_inner_states/sti_vs_search_3_run.hpp>
#include <sm_cl_px4_mr_test_4/states/vs_chain_1_inner_states/sti_vs_chain_1_run.hpp>
#include <sm_cl_px4_mr_test_4/states/vs_chain_2_inner_states/sti_vs_chain_2_run.hpp>
#include <sm_cl_px4_mr_test_4/states/vs_chain_3_inner_states/sti_vs_chain_3_run.hpp>
