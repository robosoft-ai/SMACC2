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

#include <nav2z_client/components/waypoints_navigator/cp_waypoints_navigator_base.hpp>
#include <smacc2/smacc_client_behavior.hpp>

namespace cl_nav2z
{
struct CbLoadWaypointsFile : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbLoadWaypointsFile(std::string filepath);

  CbLoadWaypointsFile(std::string parameter_name, std::string packagenamespace);

  void onEntry() override;

  void onExit() override;

  std::optional<std::string> filepath_;

  std::optional<std::string> parameterName_;
  std::optional<std::string> packageNamespace_;

  cl_nav2z::CpWaypointNavigatorBase * waypointsNavigator_;
};
}  // namespace cl_nav2z
