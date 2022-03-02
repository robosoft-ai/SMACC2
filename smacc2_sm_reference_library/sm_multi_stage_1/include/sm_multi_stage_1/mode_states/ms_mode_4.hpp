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

#include <smacc2/smacc.hpp>
namespace sm_multi_stage_1
{
// STATE DECLARATION
class MsMode4 : public smacc2::SmaccState<MsMode4, SmMultiStage1, Mode4StObserve>
{
public:
  using SmaccState::SmaccState;

// TRANSITION TABLE
typedef mpl::list<

  Transition<EvLoopEnd<Mode4SequenceALoop>, MsMode5>,
  Transition<EvLoopEnd<Mode4SequenceBLoop>, MsMode5>,
  Transition<EvLoopEnd<Mode4SequenceCLoop>, MsMode5>,
  Transition<EvLoopEnd<Mode4SequenceDLoop>, MsMode5>

    >reactions;


// STATE VARIABLES

  // AC Cycle Loop
  static constexpr int ztotal_iterations() { return 1; }
  int ziteration_count = 0;

  // CMV Cycle Loop
  static constexpr int ytotal_iterations() { return 1; }
  int yiteration_count = 0;

  // DC Cycle Loop
  static constexpr int dtotal_iterations() { return 1; }
  int diteration_count = 0;

  // GC Cycle Loop
  static constexpr int gtotal_iterations() { return 1; }
  int giteration_count = 0;

};
}  // namespace sm_multi_stage_1
