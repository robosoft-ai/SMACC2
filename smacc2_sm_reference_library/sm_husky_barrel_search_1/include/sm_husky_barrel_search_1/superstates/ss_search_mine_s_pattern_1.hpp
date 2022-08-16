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
#include <sm_husky_barrel_search_1/clients/opencv_perception_client/cl_opencv_perception_client.hpp>

namespace sm_husky_barrel_search_1
{
namespace SS5
{
namespace sm_husky_barrel_search_1
{
namespace s_pattern_states
{
// FORWARD DECLARATIONS OF INNER STATES
class StiSPatternRotate1;
class StiSPatternForward1;
class StiSPatternRotate2;
class StiSPatternForward2;
class StiSPatternRotate3;
class StiSPatternForward3;
class StiSPatternRotate4;
class StiSPatternForward4;
class StiSPatternLoopStart;
}  // namespace s_pattern_states
}  // namespace sm_husky_barrel_search_1

enum class TDirection
{
  LEFT,
  RIGHT
};

using namespace sm_husky_barrel_search_1::s_pattern_states;
using namespace cl_opencv_perception;
// STATE DECLARATION
struct SsSearchMineSPattern1 : smacc2::SmaccState<SsSearchMineSPattern1, SmHuskyBarrelSearch1, StiSPatternLoopStart>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    // Transition<EvEnemyClusterDetected<ClOpenCVPerception, OrPerception>, StFire, SUCCESS>,
    Transition<EvLoopEnd<StiSPatternLoopStart>, StNavigateToFireEnemyPosition, ENDLOOP>
    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    //configure_orthogonal<OrObstaclePerception, CbLidarSensor>();
  }

  static constexpr float pitch1_lenght_meters() { return 0.75; }
  static constexpr float pitch2_lenght_meters() { return 15.0;/*0.75;*/ }
  static constexpr int total_iterations() { return 9; }
  static constexpr TDirection direction() { return TDirection::RIGHT; }
  static constexpr float base_angle_degrees() { return 45.0; }

  int iteration_count;

  void runtimeConfigure() { this->iteration_count = 0; }
};

// FORWARD DECLARATION FOR THE SUPERSTATE
using SS = SsSearchMineSPattern1;
#include <sm_husky_barrel_search_1/states/s_pattern_states/sti_spattern_forward_1.hpp>
#include <sm_husky_barrel_search_1/states/s_pattern_states/sti_spattern_forward_2.hpp>
#include <sm_husky_barrel_search_1/states/s_pattern_states/sti_spattern_forward_3.hpp>
#include <sm_husky_barrel_search_1/states/s_pattern_states/sti_spattern_forward_4.hpp>
#include <sm_husky_barrel_search_1/states/s_pattern_states/sti_spattern_loop_start.hpp>
#include <sm_husky_barrel_search_1/states/s_pattern_states/sti_spattern_rotate_1.hpp>
#include <sm_husky_barrel_search_1/states/s_pattern_states/sti_spattern_rotate_2.hpp>
#include <sm_husky_barrel_search_1/states/s_pattern_states/sti_spattern_rotate_3.hpp>
#include <sm_husky_barrel_search_1/states/s_pattern_states/sti_spattern_rotate_4.hpp>
}  // namespace SS5
}  // namespace sm_husky_barrel_search_1
