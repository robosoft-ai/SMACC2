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
#include <angles/angles.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <forward_global_planner/forward_global_planner.hpp>
#include <move_base_z_planners_common/common.hpp>
#include <move_base_z_planners_common/move_base_z_client_tools.hpp>

#include <boost/assign.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <fstream>
#include <nav_2d_utils/tf_help.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <streambuf>

namespace cl_move_base_z
{
namespace forward_global_planner
{
ForwardGlobalPlanner::ForwardGlobalPlanner()
//   : nh_("~/ForwardGlobalPlanner")
{
  skip_straight_motion_distance_ = 0.2;  // meters
  puresSpinningRadStep_ = 1000;          // rads
}

ForwardGlobalPlanner::~ForwardGlobalPlanner() {}

void ForwardGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nh_ = parent.lock();
  tf_ = tf;
  name_ = name;
  costmap_ros_ = costmap_ros;

  RCLCPP_INFO(nh_->get_logger(), "[Forward Global Planner] initializing");
  planPub_ = nh_->create_publisher<nav_msgs::msg::Path>("global_plan", 1);
  skip_straight_motion_distance_ = 0.2;  // meters
  puresSpinningRadStep_ = 1000;          // rads
  transform_tolerance_ = 0.1;
}

void ForwardGlobalPlanner::cleanup() {}

void ForwardGlobalPlanner::activate() { planPub_->on_activate(); }

void ForwardGlobalPlanner::deactivate()
{
  nav_msgs::msg::Path planMsg;
  planPub_->publish(planMsg);
  planPub_->on_deactivate();
}

nav_msgs::msg::Path ForwardGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_INFO(nh_->get_logger(), "[Forward Global Planner] planning");

  rclcpp::Duration ttol = rclcpp::Duration::from_seconds(transform_tolerance_);
  //---------------------------------------------------------------------
  geometry_msgs::msg::PoseStamped transformedStart;
  nav_2d_utils::transformPose(tf_, costmap_ros_->getGlobalFrameID(), start, transformedStart, ttol);
  transformedStart.header.frame_id = costmap_ros_->getGlobalFrameID();
  //---------------------------------------------------------------------
  geometry_msgs::msg::PoseStamped transformedGoal;
  nav_2d_utils::transformPose(tf_, costmap_ros_->getGlobalFrameID(), goal, transformedGoal, ttol);
  transformedGoal.header.frame_id = costmap_ros_->getGlobalFrameID();
  //---------------------------------------------------------------------

  nav_msgs::msg::Path planMsg;
  std::vector<geometry_msgs::msg::PoseStamped> plan;

  // three stages: 1 - heading to goal position, 2 - going forward keep orientation, 3 - heading to goal orientation

  // 1 - heading to goal position
  // orientation direction

  double dx = transformedGoal.pose.position.x - transformedStart.pose.position.x;
  double dy = transformedGoal.pose.position.y - transformedStart.pose.position.y;

  double length = sqrt(dx * dx + dy * dy);

  geometry_msgs::msg::PoseStamped prevState;
  if (length > skip_straight_motion_distance_)
  {
    // skip initial pure spinning and initial straight motion
    // RCLCPP_INFO(nh_->get_logger(),"1 - heading to goal position pure spinning");
    double heading_direction = atan2(dy, dx);
    prevState = cl_move_base_z::makePureSpinningSubPlan(
      transformedStart, heading_direction, plan, puresSpinningRadStep_);

    // RCLCPP_INFO(nh_->get_logger(), "2 - going forward keep orientation pure straight");
    prevState = cl_move_base_z::makePureStraightSubPlan(
      prevState, transformedGoal.pose.position, length, plan);
  }
  else
  {
    prevState = transformedStart;
  }

  // RCLCPP_INFO(nh_->get_logger(),"3 - heading to goal orientation");
  double goalOrientation = angles::normalize_angle(tf2::getYaw(transformedGoal.pose.orientation));
  cl_move_base_z::makePureSpinningSubPlan(prevState, goalOrientation, plan, puresSpinningRadStep_);
  planMsg.poses = plan;
  planMsg.header.stamp = this->nh_->now();
  planMsg.header.frame_id = this->costmap_ros_->getGlobalFrameID();

  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[Forward Global Planner] generated plan size: " << plan.size());

  // check plan rejection
  bool acceptedGlobalPlan = true;

  RCLCPP_INFO(
    nh_->get_logger(), "[Forward Global Planner] checking obstacles in the generated plan");
  nav2_costmap_2d::Costmap2D * costmap2d = this->costmap_ros_->getCostmap();
  for (auto & p : plan)
  {
    unsigned int mx, my;
    costmap2d->worldToMap(p.pose.position.x, p.pose.position.y, mx, my);
    auto cost = costmap2d->getCost(mx, my);

    // static const unsigned char NO_INFORMATION = 255;
    // static const unsigned char LETHAL_OBSTACLE = 254;
    // static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
    // static const unsigned char FREE_SPACE = 0;

    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      RCLCPP_INFO_STREAM(
        nh_->get_logger(), "[Forward Global Planner] pose " << p.pose.position.x << ", "
                                                            << p.pose.position.y
                                                            << " rejected, cost: " << cost);
      acceptedGlobalPlan = false;
      break;
    }
  }

  if (acceptedGlobalPlan)
  {
    RCLCPP_INFO_STREAM(
      nh_->get_logger(), "[Forward Global Planner] accepted plan: " << plan.size());
    planPub_->publish(planMsg);
    return planMsg;
  }
  else
  {
    RCLCPP_INFO(nh_->get_logger(), "[Forward Global Planner] plan rejected");
    planMsg.poses.clear();
    planPub_->publish(planMsg);
    return planMsg;
  }
}

}  // namespace forward_global_planner
}  // namespace cl_move_base_z

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(
  cl_move_base_z::forward_global_planner::ForwardGlobalPlanner, nav2_core::GlobalPlanner)
