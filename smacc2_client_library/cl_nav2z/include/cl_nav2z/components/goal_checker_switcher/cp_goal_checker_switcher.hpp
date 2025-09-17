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
#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <smacc2/component.hpp>
#include <std_msgs/msg/string.hpp>

namespace cl_nav2z
{
// this class is used to switch the current goal checker of the remote navigation2 stack controller
class CpGoalCheckerSwitcher : public smacc2::ISmaccComponent
{
public:
  CpGoalCheckerSwitcher(
    std::string goal_checker_selector_topic = "goal_checker_selector",
    std::string default_goal_checker_name = "goal_checker")
  : goal_checker_selector_topic_(goal_checker_selector_topic),
    default_goal_checker_name_(default_goal_checker_name)
  {
  }

  void onInitialize() override
  {
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local().reliable();

    this->goal_checker_selector_pub_ =
      getNode()->create_publisher<std_msgs::msg::String>(goal_checker_selector_topic_, qos);
  }

  virtual ~CpGoalCheckerSwitcher() {}

  void setDefaultGoalChecker() { setGoalCheckerId(default_goal_checker_name_); }

  void setGoalCheckerId(std::string goalcheckerid)
  {
    RCLCPP_INFO_STREAM(
      getLogger(), "[CpGoalCheckerSwitcher] Setting goal checker: " << goalcheckerid);

    std_msgs::msg::String msg;
    msg.data = goalcheckerid;
    this->goal_checker_selector_pub_->publish(msg);
  }

private:
  std::string goal_checker_selector_topic_;
  std::string default_goal_checker_name_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_checker_selector_pub_;
};
}  // namespace cl_nav2z
