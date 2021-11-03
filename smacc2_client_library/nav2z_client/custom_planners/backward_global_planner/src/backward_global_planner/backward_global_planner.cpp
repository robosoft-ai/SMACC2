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

#include <backward_global_planner/backward_global_planner.hpp>
#include <nav2z_planners_common/common.hpp>

#include <angles/angles.h>
#include <tf2/transform_datatypes.h>
#include <nav2z_planners_common/nav2z_client_tools.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/utils.h>
#include <boost/assign.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <fstream>
#include <nav_2d_utils/tf_help.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <streambuf>

using namespace std::chrono_literals;
namespace cl_nav2z
{
namespace backward_global_planner
{
/**
******************************************************************************************************************
* Constructor()
******************************************************************************************************************
*/
BackwardGlobalPlanner::BackwardGlobalPlanner()
{
  skip_straight_motion_distance_ = 0.2;
  puresSpinningRadStep_ = 1000;  // rads
}

BackwardGlobalPlanner::~BackwardGlobalPlanner() {}

/**
******************************************************************************************************************
* initialize()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  this->nh_ = parent.lock();
  name_ = name;
  tf_ = tf;
  transform_tolerance_ = 0.1;

  // RCLCPP_INFO_NAMED(nh_->get_logger(), "Backwards", "BackwardGlobalPlanner initialize");
  costmap_ros_ = costmap_ros;
  // RCLCPP_WARN_NAMED(nh_->get_logger(), "Backwards", "initializating global planner, costmap address: %ld",
  // (long)costmap_ros);

  planPub_ = nh_->create_publisher<nav_msgs::msg::Path>("backward_planner/global_plan", 1);
  markersPub_ =
    nh_->create_publisher<visualization_msgs::msg::MarkerArray>("backward_planner/markers", 1);

  declareOrSet(nh_, name_ + ".transform_tolerance", transform_tolerance_);
}

/**
******************************************************************************************************************
* cleanup()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::cleanup() { this->cleanMarkers(); }

/**
******************************************************************************************************************
* activate()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::activate()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[BackwardGlobalPlanner] activating planner");
  planPub_->on_activate();
  markersPub_->on_activate();
}

/**
******************************************************************************************************************
* deactivate()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::deactivate()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[BackwardGlobalPlanner] deactivating planner");

  // clear
  nav_msgs::msg::Path planMsg;
  planMsg.header.stamp = nh_->now();
  planPub_->publish(planMsg);

  planPub_->on_deactivate();
  this->cleanMarkers();
  markersPub_->on_deactivate();
}

/**
******************************************************************************************************************
* publishGoalMarker()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::publishGoalMarker(
  const geometry_msgs::msg::Pose & pose, double r, double g, double b)
{
  double phi = tf2::getYaw(pose.orientation);
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = this->costmap_ros_->getGlobalFrameID();
  marker.header.stamp = nh_->now();
  marker.ns = "my_namespace2";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(0s);
  marker.scale.x = 0.1;
  marker.scale.y = 0.3;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;

  geometry_msgs::msg::Point start, end;
  start.x = pose.position.x;
  start.y = pose.position.y;

  end.x = pose.position.x + 0.5 * cos(phi);
  end.y = pose.position.y + 0.5 * sin(phi);

  marker.points.push_back(start);
  marker.points.push_back(end);

  visualization_msgs::msg::MarkerArray ma;
  ma.markers.push_back(marker);

  markersPub_->publish(ma);
}

void BackwardGlobalPlanner::cleanMarkers()
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = this->costmap_ros_->getGlobalFrameID();
  marker.header.stamp = nh_->now();
  marker.ns = "my_namespace2";
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;

  visualization_msgs::msg::MarkerArray ma;
  ma.markers.push_back(marker);

  markersPub_->publish(ma);
}

/**
******************************************************************************************************************
* defaultBackwardPath()
******************************************************************************************************************
*/
void BackwardGlobalPlanner::createDefaultBackwardPath(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
  std::vector<geometry_msgs::msg::PoseStamped> & plan)
{
  auto q = start.pose.orientation;

  geometry_msgs::msg::PoseStamped pose;
  pose = start;

  double dx = start.pose.position.x - goal.pose.position.x;
  double dy = start.pose.position.y - goal.pose.position.y;

  double length = sqrt(dx * dx + dy * dy);

  geometry_msgs::msg::PoseStamped prevState;
  if (length > skip_straight_motion_distance_)
  {
    // skip initial pure spinning and initial straight motion
    RCLCPP_INFO(
      nh_->get_logger(), "1 - heading to goal position pure spinning radstep: %lf",
      puresSpinningRadStep_);
    double heading_direction = atan2(dy, dx);
    double startyaw = tf2::getYaw(q);
    double offset = angles::shortest_angular_distance(startyaw, heading_direction);
    heading_direction = startyaw + offset;

    prevState =
      cl_nav2z::makePureSpinningSubPlan(start, heading_direction, plan, puresSpinningRadStep_);
    RCLCPP_INFO(nh_->get_logger(), "2 - going forward keep orientation pure straight");

    prevState = cl_nav2z::makePureStraightSubPlan(prevState, goal.pose.position, length, plan);
  }
  else
  {
    prevState = start;
  }

  RCLCPP_WARN_STREAM(
    nh_->get_logger(), "[BackwardGlobalPlanner] backward global plan size:  " << plan.size());
}

/**
******************************************************************************************************************
* makePlan()
******************************************************************************************************************
*/
nav_msgs::msg::Path BackwardGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardGlobalPlanner] goal frame id: "
                         << goal.header.frame_id << " pose: " << goal.pose.position);
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardGlobalPlanner] goal pose frame id: " << goal.header.frame_id);

  rclcpp::Duration ttol = rclcpp::Duration::from_seconds(transform_tolerance_);
  //---------------------------------------------------------------------
  geometry_msgs::msg::PoseStamped transformedStart;
  nav_2d_utils::transformPose(tf_, costmap_ros_->getGlobalFrameID(), start, transformedStart, ttol);
  transformedStart.header.frame_id = costmap_ros_->getGlobalFrameID();
  //---------------------------------------------------------------------
  geometry_msgs::msg::PoseStamped transformedGoal;
  nav_2d_utils::transformPose(tf_, costmap_ros_->getGlobalFrameID(), goal, transformedGoal, ttol);
  transformedGoal.header.frame_id = costmap_ros_->getGlobalFrameID();

  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardGlobalPlanner] creating default backward global plan");
  //---------------------------------------------------------------------
  std::vector<geometry_msgs::msg::PoseStamped> plan;
  this->createDefaultBackwardPath(transformedStart, transformedGoal, plan);

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[BackwardGlobalPlanner] publishing markers");
  publishGoalMarker(transformedGoal.pose, 1.0, 0, 1.0);

  nav_msgs::msg::Path planMsg;
  planMsg.poses = plan;
  planMsg.header.stamp = this->nh_->now();
  planMsg.header.frame_id = this->costmap_ros_->getGlobalFrameID();

  //---------------------------------------------------------------------
  // check plan rejection if obstacle is found
  bool acceptedGlobalPlan = true;

  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardGlobalPlanner] checking backwards trajectory on costmap");
  auto costmap2d = this->costmap_ros_->getCostmap();
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
      acceptedGlobalPlan = false;
      break;
    }
  }

  if (!acceptedGlobalPlan)
  {
    RCLCPP_WARN_STREAM(
      nh_->get_logger(),
      "[BackwardGlobalPlanner] backward plan request is not accepted, returning "
      "empty path");
    planMsg.poses.clear();
  }

  RCLCPP_WARN_STREAM(
    nh_->get_logger(), "[BackwardGlobalPlanner] backward global plan publishing path. poses count: "
                         << planMsg.poses.size());
  planPub_->publish(planMsg);

  return planMsg;
}

}  // namespace backward_global_planner
}  // namespace cl_nav2z

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(
  cl_nav2z::backward_global_planner::BackwardGlobalPlanner, nav2_core::GlobalPlanner)
