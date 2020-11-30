/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <move_base_z_client_plugin/components/goal_checker_switcher/goal_checker_switcher.h>

namespace cl_move_base_z
{
GoalCheckerSwitcher::GoalCheckerSwitcher(std::string controller_node_name, std::string default_goal_checker_name)
  : controller_node_name_(controller_node_name), default_goal_checker_name_(default_goal_checker_name)
{
}

GoalCheckerSwitcher::~GoalCheckerSwitcher()
{
}

void GoalCheckerSwitcher::onInitialize()
{
  controller_server_node_ =
      std::make_shared<rclcpp::AsyncParametersClient>(this->getNode(), this->controller_node_name_);
}

void GoalCheckerSwitcher::setDefaultGoalChecker()
{
  setGoalCheckerId(default_goal_checker_name_);  // default id in navigation2 stack
}

void GoalCheckerSwitcher::setGoalCheckerId(std::string goalcheckerid)
{
  RCLCPP_INFO_STREAM(getNode()->get_logger(), "[GoalCheckerSwitcher] Setting goal checker: " << goalcheckerid);

  controller_server_node_->wait_for_service();

  std::vector<rclcpp::Parameter> params{ rclcpp::Parameter("current_goal_checker", goalcheckerid) };

  auto futureResults = controller_server_node_->set_parameters(params);

  int i = 0;
  for (auto& res : futureResults.get())
  {
    RCLCPP_INFO_STREAM(getNode()->get_logger(), "[GoalCheckerSwitcher] parameter result: "
                                                    << params[i].get_name() << "=" << params[i].as_string()
                                                    << ". Result: " << res.successful);
    i++;

    if (!res.successful)
      RCLCPP_ERROR_STREAM(this->getNode()->get_logger(), "[GoalCheckerSwitcher] goal checker could not properly switch "
                                                         "the goal checker of the controller_server");
  }
}

}  // namespace cl_move_base_z