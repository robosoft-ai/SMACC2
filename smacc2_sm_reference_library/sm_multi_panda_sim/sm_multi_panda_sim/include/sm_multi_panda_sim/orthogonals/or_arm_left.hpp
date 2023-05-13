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

#pragma once

#include <chrono>

#include <moveit2z_client/cl_moveit2z.hpp>
#include <moveit2z_client/components/cp_trajectory_history.hpp>
#include <moveit2z_client/components/cp_grasping_objects.hpp>

#include "ros_timer_client/cl_ros_timer.hpp"
#include "smacc2/smacc.hpp"

namespace sm_multi_panda_sim
{
using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class OrArmLeft : public smacc2::Orthogonal<OrArmLeft>
{
public:
  void onInitialize() override
  {

    moveit::planning_interface::MoveGroupInterface::Options options ("left_panda_arm","robot_description_left", "/panda_arm_1");


    auto move_group_client = this->createClient<cl_moveit2z::ClMoveit2z>(options); //ur_manipulator

    move_group_client->createComponent<cl_moveit2z::CpTrajectoryHistory>();

    // auto graspingComponent = move_group_client->createComponent<cl_moveit2z::CpGraspingComponent>();
    // graspingComponent->gripperLink_="left_panda_link8";
  }
};
}  // namespace sm_multi_panda_sim
