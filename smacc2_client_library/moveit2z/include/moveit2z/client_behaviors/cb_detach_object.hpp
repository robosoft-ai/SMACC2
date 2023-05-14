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

<<<<<<< HEAD:smacc2_client_library/moveit2z_client/include/moveit2z_client/client_behaviors/cb_detach_object.hpp
#include <moveit2z_client/cl_moveit2z.hpp>
#include <moveit2z_client/components/cp_grasping_objects.hpp>
=======
#include <moveit2z/cl_moveit2z.hpp>
#include <moveit2z/components/cp_grasping_objects.hpp>
>>>>>>> 056c654b26293282493ab9a4aaec5399f25f061f:smacc2_client_library/moveit2z/include/moveit2z/client_behaviors/cb_detach_object.hpp
#include <smacc2/smacc.hpp>

namespace cl_moveit2z
{
  // releases the current attached object
  class CbDetachObject : public smacc2::SmaccAsyncClientBehavior
  {
  public:
    virtual void onEntry() override;

    virtual void onExit() override;
  };
}  // namespace cl_moveit2z
