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

namespace sm_nav2_gazebo_test_3
{
namespace primitive_loop_states
{

// STATE DECLARATION
struct StiPrimitiveLoopStart : smacc2::SmaccState<StiPrimitiveLoopStart, SS>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    Transition<EvLoopContinue<StiPrimitiveLoopStart>, StiDriveOnHeading, CONTINUELOOP>
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  bool loopWhileCondition()
  {
    auto & superstate = this->context<SS>();
    RCLCPP_INFO(
      getLogger(), "[StiPrimitiveLoopStart] current iteration: %d, total iterations: %d",
      superstate.iteration_count, superstate.total_iterations());
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry()
  {
    checkWhileLoopConditionAndThrowEvent(&StiPrimitiveLoopStart::loopWhileCondition);
  }
};

}  // namespace primitive_loop_states
}  // namespace sm_nav2_gazebo_test_3
