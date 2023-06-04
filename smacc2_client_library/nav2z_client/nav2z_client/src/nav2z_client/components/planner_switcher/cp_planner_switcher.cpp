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
#include <nav2z_client/components/planner_switcher/cp_planner_switcher.hpp>
#include <nav2z_client/nav2z_client.hpp>

namespace cl_nav2z
{
using namespace std::chrono_literals;

CpPlannerSwitcher::CpPlannerSwitcher() {}

void CpPlannerSwitcher::onInitialize()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  this->planner_selector_pub_ =
    getNode()->create_publisher<std_msgs::msg::String>("planner_selector", qos);
  this->controller_selector_pub_ =
    getNode()->create_publisher<std_msgs::msg::String>("controller_selector", qos);
}

void CpPlannerSwitcher::setDesiredGlobalPlanner(std::string plannerName)
{
  desired_planner_ = plannerName;
}

void CpPlannerSwitcher::setDesiredController(std::string controllerName)
{
  desired_controller_ = controllerName;
}

void CpPlannerSwitcher::setUndoPathBackwardPlanner(bool commit)
{
  RCLCPP_INFO(getLogger(), "[CpPlannerSwitcher] Planner Switcher: Trying to set BackwardPlanner");

  desired_planner_ = "UndoPathGlobalPlanner";
  desired_controller_ = "BackwardLocalPlanner";

  if (commit) commitPublish();
}

void CpPlannerSwitcher::setBackwardPlanner(bool commit)
{
  RCLCPP_INFO(getLogger(), "[CpPlannerSwitcher] Planner Switcher: Trying to set BackwardPlanner");

  desired_planner_ = "BackwardGlobalPlanner";
  desired_controller_ = "BackwardLocalPlanner";

  if (commit) commitPublish();
}

void CpPlannerSwitcher::setForwardPlanner(bool commit)
{
  RCLCPP_INFO(getLogger(), "[CpPlannerSwitcher] Planner Switcher: Trying to set ForwardPlanner");

  desired_planner_ = "ForwardGlobalPlanner";
  desired_controller_ = "ForwardLocalPlanner";

  if (commit) commitPublish();
}

void CpPlannerSwitcher::setPureSpinningPlanner(bool commit)
{
  RCLCPP_INFO(
    getLogger(), "[CpPlannerSwitcher] Planner Switcher: Trying to set PureSpinningPlanner");

  desired_planner_ = "ForwardGlobalPlanner";
  desired_controller_ = "PureSpinningLocalPlanner";

  if (commit) commitPublish();
}

void CpPlannerSwitcher::setDefaultPlanners(bool commit)
{
  desired_planner_ = "GridBased";
  desired_controller_ = "FollowPath";

  if (commit) commitPublish();
}

void CpPlannerSwitcher::commitPublish()
{
  std_msgs::msg::String planner_msg;
  planner_msg.data = desired_planner_;
  this->planner_selector_pub_->publish(planner_msg);

  std_msgs::msg::String controller_msg;
  controller_msg.data = desired_controller_;
  this->controller_selector_pub_->publish(controller_msg);
}
}  // namespace cl_nav2z
