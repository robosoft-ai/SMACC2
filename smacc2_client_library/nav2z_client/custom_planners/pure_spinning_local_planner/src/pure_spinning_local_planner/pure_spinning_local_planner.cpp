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

#include <angles/angles.h>
#include <nav2z_planners_common/common.hpp>
#include <pure_spinning_local_planner/pure_spinning_local_planner.hpp>

#include <nav_2d_utils/tf_help.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace cl_nav2z
{
namespace pure_spinning_local_planner
{
template <typename T>
void tryGetOrSet(rclcpp_lifecycle::LifecycleNode::SharedPtr & node, std::string param, T & value)
{
  if (!node->get_parameter(param, value))
  {
    node->set_parameter(rclcpp::Parameter(param, value));
  }
}

PureSpinningLocalPlanner::PureSpinningLocalPlanner() {}

PureSpinningLocalPlanner::~PureSpinningLocalPlanner() {}

void PureSpinningLocalPlanner::activate()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "activating controller PureSpinningLocalPlanner");
  this->updateParameters();
}

void PureSpinningLocalPlanner::deactivate() {}

void PureSpinningLocalPlanner::cleanup()
{
  this->plan_.clear();
  this->currentPoseIndex_ = 0;
  yaw_goal_tolerance_ = -1.0;
}

void PureSpinningLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & node, std::string name,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  costmapRos_ = costmap_ros;
  name_ = name;
  nh_ = node.lock();
  k_betta_ = 1.0;
  intermediate_goal_yaw_tolerance_ = 0.12;
  max_angular_z_speed_ = 1.0;
  yaw_goal_tolerance_ = -1.0;
  transform_tolerance_ = 0.1;
  tf_ = tf;

  declareOrSet(nh_, name_ + ".k_betta", k_betta_);
  declareOrSet(nh_, name_ + ".intermediate_goals_yaw_tolerance", intermediate_goal_yaw_tolerance_);
  declareOrSet(nh_, name_ + ".max_angular_z_speed", max_angular_z_speed_);
  declareOrSet(nh_, name_ + ".transform_tolerance", transform_tolerance_);
  declareOrSet(nh_, name_ + ".use_shortest_angular_distance", use_shortest_angular_distance_);

  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[PureSpinningLocalPlanner] pure spinning planner already created");
}

void PureSpinningLocalPlanner::updateParameters()
{
  tryGetOrSet(nh_, name_ + ".k_betta", k_betta_);
  tryGetOrSet(nh_, name_ + ".intermediate_goals_yaw_tolerance", intermediate_goal_yaw_tolerance_);
  tryGetOrSet(nh_, name_ + ".max_angular_z_speed", max_angular_z_speed_);
  tryGetOrSet(nh_, name_ + ".transform_tolerance", transform_tolerance_);
  tryGetOrSet(nh_, name_ + ".use_shortest_angular_distance", use_shortest_angular_distance_);
}

geometry_msgs::msg::TwistStamped PureSpinningLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * goal_checker)
{
  this->updateParameters();
  if (yaw_goal_tolerance_ == -1)
  {
    geometry_msgs::msg::Pose posetol;
    geometry_msgs::msg::Twist twistol;
    if (goal_checker->getTolerances(posetol, twistol))
    {
      yaw_goal_tolerance_ = tf2::getYaw(posetol.orientation);
      //yaw_goal_tolerance_ = tf2::getYaw(posetol.orientation) * 0.35; // WORKAROUND GOAL CHECKER DIFFERENCE NAV CONTROLLER
      RCLCPP_INFO_STREAM(
        nh_->get_logger(),
        "[PureSpinningLocalPlanner] yaw_goal_tolerance_: " << yaw_goal_tolerance_);
    }
    else
    {
      RCLCPP_INFO_STREAM(
        nh_->get_logger(), "[PureSpinningLocalPlanner] could not get tolerances from goal checker");
    }
  }

  geometry_msgs::msg::TwistStamped cmd_vel;

  goalReached_ = false;
  // RCLCPP_DEBUG(nh_->get_logger(), "LOCAL PLANNER LOOP");

  geometry_msgs::msg::PoseStamped currentPose = pose;

  tf2::Quaternion q;
  tf2::fromMsg(currentPose.pose.orientation, q);
  auto currentYaw = tf2::getYaw(currentPose.pose.orientation);
  double angular_error = 0;
  double targetYaw;

  do
  {
    auto & goal = plan_[currentPoseIndex_];
    targetYaw = tf2::getYaw(goal.pose.orientation);

    if (use_shortest_angular_distance_)
      angular_error = angles::shortest_angular_distance(currentYaw, targetYaw);
    else
      angular_error = targetYaw - currentYaw;

    // if it is in the following way, sometimes the direction is incorrect
    //angular_error = targetYaw - currentYaw;

    // all the points must be reached using the control rule, but the last one
    // have an special condition
    if ((currentPoseIndex_ < (int)plan_.size() - 1 &&
         fabs(angular_error) < this->intermediate_goal_yaw_tolerance_))
    {
      currentPoseIndex_++;
    }
    else
    {
      break;
    }
  } while (currentPoseIndex_ < (int)plan_.size());

  auto omega = angular_error * k_betta_;
  cmd_vel.twist.angular.z =
    std::min(std::max(omega, -fabs(max_angular_z_speed_)), fabs(max_angular_z_speed_));

  RCLCPP_INFO_STREAM(
    nh_->get_logger(),
    "[PureSpinningLocalPlanner] pose index: " << currentPoseIndex_ << "/" << plan_.size());
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[PureSpinningLocalPlanner] k_betta_: " << k_betta_);
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[PureSpinningLocalPlanner] angular error: " << angular_error << "(tol "
                                                                    << yaw_goal_tolerance_ << ")");
  RCLCPP_INFO_STREAM(
    nh_->get_logger(),
    "[PureSpinningLocalPlanner] command angular speed: " << cmd_vel.twist.angular.z);
  RCLCPP_INFO_STREAM(
    nh_->get_logger(),
    "[PureSpinningLocalPlanner] completion " << currentPoseIndex_ << "/" << plan_.size());

  if (currentPoseIndex_ >= (int)plan_.size() - 1 && fabs(angular_error) < yaw_goal_tolerance_)
  {
    RCLCPP_INFO_STREAM(
      nh_->get_logger(), "[PureSpinningLocalPlanner] GOAL REACHED. Sending stop command.");
    goalReached_ = true;
    cmd_vel.twist.linear.x = 0;
    cmd_vel.twist.angular.z = 0;
  }

  return cmd_vel;
}

void PureSpinningLocalPlanner::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/)
{
  RCLCPP_WARN_STREAM(
    nh_->get_logger(),
    "PureSpinningLocalPlanner::setSpeedLimit invoked. Ignored, funcionality not "
    "implemented.");
}

bool PureSpinningLocalPlanner::isGoalReached() { return goalReached_; }

void PureSpinningLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "activating controller PureSpinningLocalPlanner");

  nav_msgs::msg::Path transformedPlan;

  rclcpp::Duration ttol = rclcpp::Duration::from_seconds(transform_tolerance_);
  // transform global plan
  for (auto & p : path.poses)
  {
    geometry_msgs::msg::PoseStamped transformedPose;
    nav_2d_utils::transformPose(tf_, costmapRos_->getGlobalFrameID(), p, transformedPose, ttol);
    transformedPose.header.frame_id = costmapRos_->getGlobalFrameID();
    transformedPlan.poses.push_back(transformedPose);
  }

  plan_ = transformedPlan.poses;

  goalReached_ = false;
  currentPoseIndex_ = 0;
}

void publishGoalMarker(double /*x*/, double /*y*/, double /*phi*/) {}
}  // namespace pure_spinning_local_planner
}  // namespace cl_nav2z

PLUGINLIB_EXPORT_CLASS(
  cl_nav2z::pure_spinning_local_planner::PureSpinningLocalPlanner, nav2_core::Controller)
