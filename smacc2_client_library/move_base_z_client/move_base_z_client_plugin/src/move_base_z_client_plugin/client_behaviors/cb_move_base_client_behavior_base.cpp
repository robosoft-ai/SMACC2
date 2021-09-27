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
#include <move_base_z_client_plugin/client_behaviors/cb_move_base_client_behavior_base.hpp>

namespace cl_move_base_z
{
CbMoveBaseClientBehaviorBase::~CbMoveBaseClientBehaviorBase() {}

void CbMoveBaseClientBehaviorBase::propagateSuccessEvent(ClMoveBaseZ::WrappedResult &)
{
  this->postSuccessEvent();
}
void CbMoveBaseClientBehaviorBase::propagateFailureEvent(ClMoveBaseZ::WrappedResult &)
{
  this->postFailureEvent();
}
}  // namespace cl_move_base_z