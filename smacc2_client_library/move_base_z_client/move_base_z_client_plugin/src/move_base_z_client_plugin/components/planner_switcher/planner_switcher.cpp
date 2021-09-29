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
#include <move_base_z_client_plugin/components/planner_switcher/planner_switcher.hpp>
#include <move_base_z_client_plugin/move_base_z_client_plugin.hpp>

namespace cl_move_base_z
{
using namespace std::chrono_literals;

PlannerSwitcher::PlannerSwitcher() {}

void PlannerSwitcher::onInitialize()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  this->planner_selector_pub_ =
    getNode()->create_publisher<std_msgs::msg::String>("planner_selector", qos);
  this->controller_selector_pub_ =
    getNode()->create_publisher<std_msgs::msg::String>("controller_selector", qos);
}

void PlannerSwitcher::setUndoPathBackwardPlanner()
{
  RCLCPP_INFO(getLogger(), "[PlannerSwitcher] Planner Switcher: Trying to set BackwardPlanner");

  desired_global_planner_ = "UndoPathGlobalPlanner";
  desired_local_planner_ = "BackwardLocalPlanner";

  updatePlanners();
}

void PlannerSwitcher::setBackwardPlanner()
{
  RCLCPP_INFO(getLogger(), "[PlannerSwitcher] Planner Switcher: Trying to set BackwardPlanner");

  desired_global_planner_ = "BackwardGlobalPlanner";
  desired_local_planner_ = "BackwardLocalPlanner";
  updatePlanners();
}

void PlannerSwitcher::setForwardPlanner()
{
  RCLCPP_INFO(getLogger(), "[PlannerSwitcher] Planner Switcher: Trying to set ForwardPlanner");

  desired_global_planner_ = "ForwardGlobalPlanner";
  desired_local_planner_ = "ForwardLocalPlanner";
  updatePlanners();
}

void PlannerSwitcher::setPureSpinningPlanner()
{
  RCLCPP_INFO(getLogger(), "[PlannerSwitcher] Planner Switcher: Trying to set PureSpinningPlanner");

  desired_global_planner_ = "ForwardGlobalPlanner";
  desired_local_planner_ = "PureSpinningLocalPlanner";
  updatePlanners();
}

void PlannerSwitcher::setDefaultPlanners()
{
  desired_global_planner_ = "GridBased";
  desired_local_planner_ = "FollowPath";

  updatePlanners();
}

void PlannerSwitcher::updatePlanners()
{
  std_msgs::msg::String planner_msg;
  planner_msg.data = desired_global_planner_;
  this->planner_selector_pub_->publish(planner_msg);

  std_msgs::msg::String controller_msg;
  controller_msg.data = desired_local_planner_;
  this->controller_selector_pub_->publish(controller_msg);
}
}  // namespace cl_move_base_z
