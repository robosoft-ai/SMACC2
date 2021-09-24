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

namespace sm_dance_bot
{
namespace f_pattern_states
{
// STATE DECLARATION
template <typename SS>
struct StiFPatternForward1 : public smacc2::SmaccState<StiFPatternForward1<SS>, SS>
{
  typedef SmaccState<StiFPatternForward1<SS>, SS> TSti;
  using TSti::context_type;
  using TSti::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvCbSuccess<CbNavigateForward, OrNavigation>, StiFPatternReturn1<SS>>

    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    TSti::template configure_orthogonal<OrNavigation, CbNavigateForward>(SS::ray_lenght_meters());
    TSti::template configure_orthogonal<OrLED, CbLEDOn>();
  }

  void runtimeConfigure()
  {
    // auto &superstate = TSti::template context<SS>();
    // RCLCPP_INFO(this->getLogger(),"[SsrFpattern] Fpattern rotate: SS current iteration: %d/%d",
    // superstate.iteration_count, SS::total_iterations());
  }
};
}  // namespace f_pattern_states
}  // namespace sm_dance_bot
