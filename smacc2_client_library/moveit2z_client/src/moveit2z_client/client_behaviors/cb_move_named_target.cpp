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

<<<<<<< HEAD:smacc2_client_library/moveit2z_client/src/moveit2z_client/client_behaviors/cb_move_named_target.cpp
#include <moveit2z_client/client_behaviors/cb_move_named_target.hpp>
=======
#include <moveit2z_client/client_behaviors/cb_move_named_target.hpp>
>>>>>>> 056c654b26293282493ab9a4aaec5399f25f061f:smacc2_client_library/moveit2z_client/src/moveit2z_client/client_behaviors/cb_move_named_target.cpp

namespace cl_moveit2z
{
  CbMoveNamedTarget::CbMoveNamedTarget(std::string namedtarget) : namedTarget_(namedtarget) {}

  void CbMoveNamedTarget::onEntry()
  {
    this->requiresClient(movegroupClient_);
    movegroupClient_->moveGroupClientInterface->setNamedTarget(this->namedTarget_);
  }

  void CbMoveNamedTarget::onExit() {}

  std::map<std::string, double> CbMoveNamedTarget::getNamedTargetValues()
  {
    return movegroupClient_->moveGroupClientInterface->getNamedTargetValues(this->namedTarget_);
  }
}  // namespace cl_moveit2z
