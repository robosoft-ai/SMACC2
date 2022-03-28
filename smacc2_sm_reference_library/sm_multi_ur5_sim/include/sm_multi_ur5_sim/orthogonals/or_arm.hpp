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

#include <move_group_interface_client/cl_movegroup.hpp>
#include <move_group_interface_client/components/cp_trajectory_history.hpp>
#include <move_group_interface_client/components/cp_grasping_objects.hpp>

#include "ros_timer_client/cl_ros_timer.hpp"
#include "smacc2/smacc.hpp"

namespace sm_multi_ur5_sim
{
using namespace std::chrono_literals;

class OrArm : public smacc2::Orthogonal<OrArm>
{
public:
  void onInitialize() override
  {
    //moveit::planning_interface::MoveGroupInterface::Options options ("ur5_1ur_manipulator", moveit::planning_interface::MoveGroupInterface::ROBOT_DESCRIPTION, "ur5_1");

    //getNode()->declare_parameter("manipulator_id"); // ur5_1
    //auto prefix = getNode()->get_parameter("manipulator_id").value_to_string(); // ur5_1
    std::string prefix = "ur5_2";
    moveit::planning_interface::MoveGroupInterface::Options options (prefix + "ur_manipulator");

    auto move_group_client = this->createClient<cl_move_group_interface::ClMoveGroup>(options); //ur_manipulator
    move_group_client->createComponent<cl_move_group_interface::CpTrajectoryHistory>();
    auto graspingComponent = move_group_client->createComponent<cl_move_group_interface::CpGraspingComponent>();

    graspingComponent->gripperLink_="tool0";
    graspingComponent->createGraspableBox("virtualBox", 0,0.5,0.5,0.1,0.1,0.1);
  }
};
}  // namespace sm_multi_ur5_sim
