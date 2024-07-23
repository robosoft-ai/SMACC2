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
#pragma once
#include <nav2z_client/client_behaviors/cb_position_control_free_space.hpp>
#include <nav2z_client/components/waypoints_navigator/cp_waypoints_navigator_base.hpp>

namespace cl_nav2z
{

class CbNavigateNextWaypointFree : public cl_nav2z::CbPositionControlFreeSpace
{
public:
  CbNavigateNextWaypointFree();

  virtual ~CbNavigateNextWaypointFree();

  void onEntry() override;

  void onSucessCallback();

  void onExit() override;

protected:
  cl_nav2z::CpWaypointNavigatorBase * waypointsNavigator_;
};

}  // namespace cl_nav2z
