// Copyright 2026 RobosoftAI Inc.
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

#include <cl_nav2z/cl_nav2z.hpp>
#include <cl_nav2z/components/amcl/cp_amcl.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/drive_on_heading.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <smacc2/client_core_components/cp_action_client.hpp>
#include <smacc2/smacc_orthogonal.hpp>

namespace sm_nav2_gazebo_test_3
{

class OrNavigation : public smacc2::Orthogonal<OrNavigation>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<cl_nav2z::ClNav2Z>();

    // CpAmcl: initial pose for localization
    client->createComponent<cl_nav2z::CpAmcl>();

    // Behavior server action clients: the whole motion mission runs on these
    // primitives through CbActionClientBehaviorBase (no planner/controller
    // stack involved after bring-up)
    client->createComponent<
      smacc2::client_core_components::CpActionClient<nav2_msgs::action::Spin>>("/spin");
    client->createComponent<
      smacc2::client_core_components::CpActionClient<nav2_msgs::action::BackUp>>("/backup");
    client->createComponent<smacc2::client_core_components::CpActionClient<
      nav2_msgs::action::DriveOnHeading>>("/drive_on_heading");
  }
};

}  // namespace sm_nav2_gazebo_test_3
