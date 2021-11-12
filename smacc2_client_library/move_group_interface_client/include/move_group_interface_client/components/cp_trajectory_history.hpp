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
 *****************************************************************************************************************/

#pragma once

#include <smacc2/component.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace cl_move_group_interface
{
struct TrajectoryHistoryEntry
{
  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes result;
  std::string name;
};

class CpTrajectoryHistory : public smacc2::ISmaccComponent
{
public:
  bool getLastTrajectory(int backIndex, moveit_msgs::msg::RobotTrajectory & trajectory);

  bool getLastTrajectory(moveit_msgs::msg::RobotTrajectory & trajectory);

  void pushTrajectory(
    std::string name, const moveit_msgs::msg::RobotTrajectory & trajectory,
    moveit_msgs::msg::MoveItErrorCodes result);

private:
  std::vector<TrajectoryHistoryEntry> trajectoryHistory_;
};
}  // namespace cl_move_group_interface
