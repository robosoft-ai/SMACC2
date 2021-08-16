/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cl_move_base_z
{
geometry_msgs::msg::PoseStamped makePureSpinningSubPlan(
  const geometry_msgs::msg::PoseStamped & start, double dstRads,
  std::vector<geometry_msgs::msg::PoseStamped> & plan, double radstep = 0.005);

geometry_msgs::msg::PoseStamped makePureStraightSubPlan(
  const geometry_msgs::msg::PoseStamped & startOrientedPose, const geometry_msgs::msg::Point & goal,
  double lenght, std::vector<geometry_msgs::msg::PoseStamped> & plan);

}  // namespace cl_move_base_z