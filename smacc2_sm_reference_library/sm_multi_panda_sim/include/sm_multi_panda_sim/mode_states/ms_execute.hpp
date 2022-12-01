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

namespace sm_multi_panda_sim
{
// STATE DECLARATION
struct MsExecute : smacc2::SmaccState<MsExecute, DsRun, StAcquireSensors>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<
    smacc2::Transition<EvSc, MsCompleting, SUCCESS>,
    smacc2::Transition<EvSuspend, MsSuspending, SUCCESS>,
    smacc2::Transition<EvHold, MsHolding, SUCCESS>

    >reactions;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}


  static constexpr int ztotal_iterations() { return 1; }
  int ziteration_count = 0;


  static constexpr int ytotal_iterations() { return 1; }
  int yiteration_count = 0;
};

}  // namespace sm_multi_panda_sim
