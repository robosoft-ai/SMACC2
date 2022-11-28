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

#include <nav2z_client/client_behaviors/cb_seek_waypoint.hpp>

namespace cl_nav2z
{
CbSeekWaypoint::CbSeekWaypoint(std::string seekWaypointName)
: count_(std::nullopt), seekWaypointName_(seekWaypointName)
{
}

CbSeekWaypoint::~CbSeekWaypoint() {}

void CbSeekWaypoint::onEntry()
{
  cl_nav2z::ClNav2Z * nav2zClient_;
  this->requiresClient(nav2zClient_);
  waypointsNavigator_ = nav2zClient_->getComponent<WaypointNavigator>();

  if (count_)
  {
    waypointsNavigator_->forward(*count_);
    count_ = std::nullopt;
  }
  else if (seekWaypointName_)
  {
    waypointsNavigator_->seekName(*seekWaypointName_);
    seekWaypointName_ = std::nullopt;
  }
}

void CbSeekWaypoint::onExit() {}

}  // namespace cl_nav2z
