/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/client_bases/smacc_action_client.h>
#include <smacc/component.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace cl_move_base_z
{
// this component is used to switch the current planner and controller interacting
// with the remote navigation2 stack nodes (bt_navigator, planner_server, controller_server)
class PlannerSwitcher : public smacc::ISmaccComponent
{
public:
  PlannerSwitcher();

  void setBackwardPlanner();

  void setUndoPathBackwardPlanner();

  void setForwardPlanner();

  void setPureSpinningPlanner();

  void onInitialize() override;

  // sets ROS defaults local and global planners
  void setDefaultPlanners();

private:
  std::string desired_global_planner_;

  std::string desired_local_planner_;

  bool set_planners_mode_flag_;

  void updatePlanners();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_selector_pub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_selector_pub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_checker_selector_pub_;
};
}  // namespace cl_move_base_z
