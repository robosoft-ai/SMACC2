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

#include <memory>
#include <string>

#include <nav2_core/global_planner.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cl_nav2z
{
namespace forward_global_planner
{
class ForwardGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<ForwardGlobalPlanner>;

  ForwardGlobalPlanner();

  /**
   * @brief Virtual destructor
   */
  virtual ~ForwardGlobalPlanner();

  /**
   * @param  parent pointer to user's node
   * @param  name The name of this planner
   * @param  tf A pointer to a TF buffer
   * @param  costmap_ros A pointer to the costmap
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup();

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate();

  /**
   * @brief Method to deactivate planner and any threads involved in execution.
   */
  virtual void deactivate();

  /**
   * @brief Method create the plan from a starting and ending goal.
   * @param start The starting pose of the robot
   * @param goal  The goal pose of the robot
   * @return      The sequence of poses to get from start to goal, if any
   */
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal);

private:
  // rclcpp::Node::SharedPtr nh_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr nh_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> planPub_;

  /// stored but almost not used
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  double skip_straight_motion_distance_;  // meters

  double puresSpinningRadStep_;  // rads

  std::string name_;

  double transform_tolerance_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
};
}  // namespace forward_global_planner
}  // namespace cl_nav2z
