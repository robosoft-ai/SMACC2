/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <move_base_z_client_plugin/components/goal_checker_switcher/goal_checker_switcher.h>

namespace cl_move_base_z
{
GoalCheckerSwitcher::GoalCheckerSwitcher(std::string goal_checker_selector_topic, std::string default_goal_checker_name)
  : goal_checker_selector_topic_(goal_checker_selector_topic), default_goal_checker_name_(default_goal_checker_name)
{
}

GoalCheckerSwitcher::~GoalCheckerSwitcher()
{
}

void GoalCheckerSwitcher::onInitialize()
{
  this->goal_checker_selector_pub_ =
      getNode()->create_publisher<std_msgs::msg::String>(goal_checker_selector_topic_, 10);
}

void GoalCheckerSwitcher::setDefaultGoalChecker()
{
  setGoalCheckerId(default_goal_checker_name_);  // default id in navigation2 stack
}

void GoalCheckerSwitcher::setGoalCheckerId(std::string goalcheckerid)
{
  RCLCPP_INFO_STREAM(getNode()->get_logger(), "[GoalCheckerSwitcher] Setting goal checker: " << goalcheckerid);

  // controller_server_node_->wait_for_service();
  // std::vector<rclcpp::Parameter> params{ rclcpp::Parameter("current_goal_checker", goalcheckerid) };
  // auto futureResults = controller_server_node_->set_parameters(params);

  std_msgs::msg::String msg;
  msg.data = goalcheckerid;
  this->goal_checker_selector_pub_->publish(msg);

  // int i = 0;
  // for (auto& res : futureResults.get())
  // {
  //   RCLCPP_INFO_STREAM(getNode()->get_logger(), "[GoalCheckerSwitcher] parameter result: "
  //                                                   << params[i].get_name() << "=" << params[i].as_string()
  //                                                   << ". Result: " << res.successful);
  //   i++;

  //   if (!res.successful)
  //     RCLCPP_ERROR_STREAM(this->getNode()->get_logger(), "[GoalCheckerSwitcher] goal checker could not properly
  //     switch "
  //                                                        "the goal checker of the controller_server");
  // }
}

}  // namespace cl_move_base_z