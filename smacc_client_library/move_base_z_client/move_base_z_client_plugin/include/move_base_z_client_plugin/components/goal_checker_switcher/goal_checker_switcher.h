/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/component.h>

#include <rclcpp/rclcpp.hpp>

namespace cl_move_base_z
{
// this class is used to switch the current goal checker of the remote navigation2 stack controller
class GoalCheckerSwitcher : public smacc::ISmaccComponent
{
public:
  GoalCheckerSwitcher(std::string controller_node_name = "/controller_server",
                      std::string default_goal_checker_name = "goal_checker");
  void onInitialize() override;
  virtual ~GoalCheckerSwitcher();
  void setDefaultGoalChecker();
  void setGoalCheckerId(std::string goal_checker_id);

private:
  std::string controller_node_name_;
  std::string default_goal_checker_name_;
  rclcpp::AsyncParametersClient::SharedPtr controller_server_node_;
};
}  // namespace cl_move_base_z
