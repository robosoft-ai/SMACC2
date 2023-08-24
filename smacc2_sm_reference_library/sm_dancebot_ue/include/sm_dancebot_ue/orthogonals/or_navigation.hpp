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

#include <smacc2/smacc_orthogonal.hpp>

#include <nav2z_client/components/waypoints_navigator/cp_waypoints_navigator_base.hpp>
#include <sm_dancebot_ue/clients/components/cp_ue_pose.hpp>

namespace sm_dancebot_ue
{
using namespace ::cl_nav2z;
using namespace std::chrono_literals;

class OrNavigation : public smacc2::Orthogonal<OrNavigation>
{
public:
  void onInitialize() override
  {
    auto nav2zClient = this->createClient<smacc2::ISmaccClient>();

    nav2zClient->createComponent<sm_dancebot_ue::CpUEPose>("/ue_ros/map_origin_entity_state");

    /*auto waypointsNavigator = */nav2zClient->createComponent<::cl_nav2z::CpWaypointNavigatorBase>();
  }
};
}  // namespace sm_dancebot_ue
