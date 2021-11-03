/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
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
