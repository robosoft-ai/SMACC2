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

namespace sm_multi_stage_1
{
namespace a_sequence_2
{
// STATE DECLARATION
struct StiASequenceLoop2 : smacc2::SmaccState<StiASequenceLoop2, SsASequence2>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvLoopContinue<StiASequenceLoop2>, StiASequenceStep12, CONTINUELOOP>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}

  bool loopWhileCondition()
  {
    auto & superstate = this->context<SsASequence2>();

    RCLCPP_INFO(
      getLogger(), "Loop start, current iterations: %d, total iterations: %d",
      superstate.iteration_count, superstate.total_iterations());
    return superstate.iteration_count++ < superstate.total_iterations();
  }

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "LOOP START ON ENTRY");
    checkWhileLoopConditionAndThrowEvent(&StiASequenceLoop2::loopWhileCondition);
  }
};
}  // namespace a_sequence_1
}  // namespace sm_multi_stage_1
