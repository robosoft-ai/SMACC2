// Copyright 2024 Robosoft Inc.
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

#pragma once
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <smacc2/client_bases/smacc_action_client_base.hpp>
#include <smacc2/smacc.hpp>

namespace cl_nav2z
{
// Declare Client class, inherit from SmaccActionClientBase template with default action argument.
class ClNav2Z
: public smacc2::client_bases::SmaccActionClientBase<nav2_msgs::action::NavigateToPose>
{
public:
  // Bring the SmaccActionClientBase types into the current scope.
  using smacc2::client_bases::SmaccActionClientBase<nav2_msgs::action::NavigateToPose>::GoalHandle;
  using smacc2::client_bases::SmaccActionClientBase<
    nav2_msgs::action::NavigateToPose>::ResultCallback;

  // Define "SmaccNavigateResultSignal" as a SmaccSignal that points to the WrappedResult type of SmaccActionClientBase.
  typedef smacc2::SmaccSignal<void(const WrappedResult &)> SmaccNavigateResultSignal;

  // Declare Client class constructor with string set to the right ROS action.
  ClNav2Z(std::string navigateToPoseAction = "/navigate_to_pose");

  // Declare Client class destructor.
  virtual ~ClNav2Z();
};

}  // namespace cl_nav2z
