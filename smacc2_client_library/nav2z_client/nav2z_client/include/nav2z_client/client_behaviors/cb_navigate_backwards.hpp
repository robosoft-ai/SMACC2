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

#include <nav2z_client/components/odom_tracker/odom_tracker.hpp>

#include <optional>

#include <nav2z_client/components/waypoints_navigator/waypoints_navigator.hpp>
#include "cb_nav2z_client_behavior_base.hpp"

namespace cl_nav2z
{
// It sends the mobile base some distance backwards
class CbNavigateBackwards : public CbNav2ZClientBehaviorBase
{
public:
  float backwardDistance;

  // just a stub to show how to use parameterless constructor
  std::optional<float> backwardSpeed;

  std::optional<std::string> goalChecker_;

  cl_nav2z::odom_tracker::OdomTracker * odomTracker_;

  CbNavigateBackwards(float backwardDistance);

  void onEntry() override;

  void onExit() override;
};
}  // namespace cl_nav2z
