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

namespace sm_three_some
{
// STATE DECLARATION
struct StState4 : smacc2::SmaccState<StState4, MsRun>
{
  using SmaccState::SmaccState;

  // DECLARE CUSTOM OBJECT TAGS
  struct TIMEOUT : ABORT{};
  struct NEXT : SUCCESS{};

  typedef mpl::list<Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, SS1::Ss1, TIMEOUT>>transitions;

  struct PREVIOUS : ABORT{};

  // STATE FUNCTIONS
  static void staticConfigure() { configure_orthogonal<OrTimer, CbTimerCountdownOnce>(10); }

  void runtimeConfigure() {}

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_three_some
