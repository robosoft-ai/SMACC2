// Copyright 2025 Robosoft Inc.
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

#ifndef ROBOT_STATE_MACHINE__STATES__ST_STATE_1_HPP_
#define ROBOT_STATE_MACHINE__STATES__ST_STATE_1_HPP_

#include "smacc2/smacc.hpp"

namespace robot_state_machine
{

//STATE DECLARATION
struct StState1 : smacc2::SmaccState<StState1, SmSimpleActionClient>
{
  using SmaccState::SmaccState;

  //Declare custom object tags
  struct AUTONOMOUS_MODE : SUCCESS
  {
  };
  struct MANUAL_MODE : SUCCESS
  {
  };

  //TRANSITION TABLE
  typedef mpl::list<
    smacc2::Transition<EvAutonomousMode<ClModeSelect, OrModeSelect>, StState2, AUTONOMOUS_MODE> >
    reactions;

  //state functions
  static void staticConfigure() { configure_orthogonal<OrModeSelect, CbModeSelect>(); }

  void runtimeConfigure() {}
};
}  // namespace robot_state_machine

#endif  // ROBOT_STATE_MACHINE__STATES__ST_STATE_1_HPP_
