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
 *****************************************************************************************************************/

#pragma once

#include <map>
#include <string>

<<<<<<< HEAD:smacc2_client_library/moveit2z_client/include/moveit2z_client/client_behaviors/cb_move_named_target.hpp
#include <moveit2z_client/cl_moveit2z.hpp>
=======
#include <moveit2z/cl_moveit2z.hpp>
>>>>>>> 056c654b26293282493ab9a4aaec5399f25f061f:smacc2_client_library/moveit2z/include/moveit2z/client_behaviors/cb_move_named_target.hpp
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_moveit2z
{
//named targets are configured in the urdf file
class CbMoveNamedTarget : public smacc2::SmaccAsyncClientBehavior
{
protected:
  ClMoveit2z * movegroupClient_;
  std::string namedTarget_;

public:
  CbMoveNamedTarget(std::string namedtarget);

  virtual void onEntry() override;

  virtual void onExit() override;

  std::map<std::string, double> getNamedTargetValues();
};
}  // namespace cl_moveit2z
