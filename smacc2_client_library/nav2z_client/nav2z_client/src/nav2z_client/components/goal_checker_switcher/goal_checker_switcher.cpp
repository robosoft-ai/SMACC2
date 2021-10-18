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
 ******************************************************************************************************************/
#include <nav2z_client/components/goal_checker_switcher/goal_checker_switcher.hpp>

namespace cl_nav2z
{
GoalCheckerSwitcher::GoalCheckerSwitcher(
  std::string goal_checker_selector_topic, std::string default_goal_checker_name)
: goal_checker_selector_topic_(goal_checker_selector_topic),
  default_goal_checker_name_(default_goal_checker_name)
{
}

GoalCheckerSwitcher::~GoalCheckerSwitcher() {}

void GoalCheckerSwitcher::onInitialize()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  this->goal_checker_selector_pub_ =
    getNode()->create_publisher<std_msgs::msg::String>(goal_checker_selector_topic_, qos);
}

void GoalCheckerSwitcher::setDefaultGoalChecker()
{
  setGoalCheckerId(default_goal_checker_name_);  // default id in navigation2 stack
}

void GoalCheckerSwitcher::setGoalCheckerId(std::string goalcheckerid)
{
  RCLCPP_INFO_STREAM(getLogger(), "[GoalCheckerSwitcher] Setting goal checker: " << goalcheckerid);

  // controller_server_node_->wait_for_service();
  // std::vector<rclcpp::Parameter> params{ rclcpp::Parameter("current_goal_checker", goalcheckerid) };
  // auto futureResults = controller_server_node_->set_parameters(params);

  std_msgs::msg::String msg;
  msg.data = goalcheckerid;
  this->goal_checker_selector_pub_->publish(msg);

  // int i = 0;
  // for (auto& res : futureResults.get())
  // {
  //   RCLCPP_INFO_STREAM(getLogger(), "[GoalCheckerSwitcher] parameter result: "
  //                                                   << params[i].get_name() << "=" << params[i].as_string()
  //                                                   << ". Result: " << res.successful);
  //   i++;

  //   if (!res.successful)
  //     RCLCPP_ERROR_STREAM(this->getLogger(), "[GoalCheckerSwitcher] goal checker could not properly
  //     switch "
  //                                                        "the goal checker of the controller_server");
  // }
}

}  // namespace cl_nav2z
