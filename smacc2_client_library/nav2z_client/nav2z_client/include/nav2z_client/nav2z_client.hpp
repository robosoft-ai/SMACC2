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

#include <smacc2/client_bases/smacc_action_client_base.hpp>
#include <smacc2/smacc.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
//#include <nav2z_client/components/planner_switcher/planner_switcher.hpp>

namespace cl_nav2z
{
class ClNav2Z
: public smacc2::client_bases::SmaccActionClientBase<nav2_msgs::action::NavigateToPose>
{
public:
  using smacc2::client_bases::SmaccActionClientBase<nav2_msgs::action::NavigateToPose>::GoalHandle;

  ClNav2Z(std::string navigateToPoseAction = "/navigate_to_pose");

  virtual ~ClNav2Z();
};

}  // namespace cl_nav2z
