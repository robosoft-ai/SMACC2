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
using namespace sm_multi_panda_sim::suspending_sequence_b;

// STATE DECLARATION
struct SsSuspendingSequenceB : smacc2::SmaccState<SsSuspendingSequenceB, MsSuspending, StiSuspendingSequenceB>
{
public:
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

    Transition<EvLoopEnd<StiSuspendingSequenceB>, SuspendingStObserve>

    >reactions;

  // STATE VARIABLES
  static constexpr int total_iterations() { return 1; }
  int iteration_count = 0;

  // STATE FUNCTIONS
  static void staticConfigure() {}

  void runtimeConfigure() {}
};  // namespace SS4

}  // namespace sm_multi_panda_sim
