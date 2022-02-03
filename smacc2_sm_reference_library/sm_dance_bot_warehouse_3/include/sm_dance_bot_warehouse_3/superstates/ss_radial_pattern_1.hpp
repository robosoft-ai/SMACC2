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

namespace sm_dance_bot_warehouse_3
{
namespace SS1
{
namespace sm_dance_bot_warehouse_3
{
namespace radial_motion_states
{
//FORWARD DECLARATION OF INNER STATES
class StiRadialRotate;
class StiRadialReturn;
class StiRadialEndPoint;
class StiRadialLoopStart;
class StiRadialEndPointRecovery;

}  // namespace radial_motion_states
}  // namespace sm_dance_bot_warehouse_3
using namespace sm_dance_bot_warehouse_3::radial_motion_states;

// STATE DECLARATION
struct SsRadialPattern1
: smacc2::SmaccState<SsRadialPattern1, MsDanceBotRunMode, StiRadialLoopStart>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    // Transition<EvLoopEnd<StiRadialLoopStart>, StRotateDegrees1, ENDLOOP>
    Transition<EvLoopEnd<StiRadialLoopStart>, StNavigateToWaypointsX, ENDLOOP>

    >reactions;

  static constexpr int total_iterations() { return 35; }
  static constexpr float ray_angle_increment_degree() { return 360.0 / total_iterations(); }
  static constexpr float ray_length_meters() { return 4; }

  int iteration_count = 0;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }

  void runtimeConfigure() {}
};

// FORWARD DECLARATION FOR THE SUPERSTATE
using SS = SsRadialPattern1;
#include <sm_dance_bot_warehouse_3/states/radial_motion_states/sti_radial_end_point.hpp>
#include <sm_dance_bot_warehouse_3/states/radial_motion_states/sti_radial_end_point_recovery.hpp>
#include <sm_dance_bot_warehouse_3/states/radial_motion_states/sti_radial_loop_start.hpp>
#include <sm_dance_bot_warehouse_3/states/radial_motion_states/sti_radial_return.hpp>
#include <sm_dance_bot_warehouse_3/states/radial_motion_states/sti_radial_rotate.hpp>

}  // namespace SS1
}  // namespace sm_dance_bot_warehouse_3
