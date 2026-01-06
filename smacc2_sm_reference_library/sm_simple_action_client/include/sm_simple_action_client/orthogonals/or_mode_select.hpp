// Copyright 2025 Robosoft Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
*

Author: Sukhraj Klair
******************************************************************************************************************/

#ifndef ROBOT_STATE_MACHINE__ORTHOGONALS__OR_MODE_SELECT_HPP_
#define ROBOT_STATE_MACHINE__ORTHOGONALS__OR_MODE_SELECT_HPP_

#include <sm_simple_action_client/mode_selection_client/cl_mode_select.hpp>
#include <smacc2/smacc_orthogonal.hpp>

namespace robot_state_machine
{
class OrModeSelect : public smacc2::Orthogonal<OrModeSelect>
{
public:
  void onInitialize() override { auto client = this->createClient<ClModeSelect>(); }
};
}  // namespace robot_state_machine

#endif  // ROBOT_STATE_MACHINE__ORTHOGONALS__OR_MODE_SELECT_HPP_
