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

#include <boost/assign.hpp>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <nav2z_planners_common/common.hpp>
#include <nav_2d_utils/tf_help.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <undo_path_global_planner/undo_path_global_planner.hpp>

// register this planner as a BaseGlobalPlanner plugin
namespace cl_nav2z
{
namespace undo_path_global_planner
{
using namespace std::chrono_literals;

/**
 ******************************************************************************************************************
 * Constructor()
 ******************************************************************************************************************
 */
UndoPathGlobalPlanner::UndoPathGlobalPlanner()
{
  skip_straight_motion_distance_ = 0.2;
  transform_tolerance_ = 0.1;
}

UndoPathGlobalPlanner::~UndoPathGlobalPlanner()
{
  // clear "rviz"- publish empty path
  nav_msgs::msg::Path planMsg;
  planMsg.header.stamp = this->nh_->now();
  planPub_->publish(planMsg);
}

void UndoPathGlobalPlanner::cleanup() { this->clearGoalMarker(); }

void UndoPathGlobalPlanner::activate()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "activating planner UndoPathGlobalPlanner");
  planPub_->on_activate();
  markersPub_->on_activate();
}

void UndoPathGlobalPlanner::deactivate()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "deactivating planner UndoPathGlobalPlanner");
  this->clearGoalMarker();
  planPub_->on_deactivate();
  markersPub_->on_deactivate();
}

/**
 ******************************************************************************************************************
 * initialize()
 ******************************************************************************************************************
 */
void UndoPathGlobalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nh_ = parent.lock();
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  name_ = name;
  // RCLCPP_WARN_NAMED(nh_->get_logger(), "Backwards", "initializating global planner, costmap address: %ld",
  // (long)costmap_ros);

  rclcpp::SensorDataQoS qos;
  qos.keep_last(2);
  forwardPathSub_ = nh_->create_subscription<nav_msgs::msg::Path>(
    "odom_tracker_path", qos,
    std::bind(&UndoPathGlobalPlanner::onForwardTrailMsg, this, std::placeholders::_1));

  planPub_ = nh_->create_publisher<nav_msgs::msg::Path>("undo_path_planner/global_plan", 1);
  markersPub_ =
    nh_->create_publisher<visualization_msgs::msg::MarkerArray>("undo_path_planner/markers", 1);

  declareOrSet(nh_, name_ + ".transform_tolerance", transform_tolerance_);
}
/**
 ******************************************************************************************************************
 * onForwardTrailMsg()
 ******************************************************************************************************************
 */
void UndoPathGlobalPlanner::onForwardTrailMsg(const nav_msgs::msg::Path::SharedPtr forwardPath)
{
  lastForwardPathMsg_ = *forwardPath;
  RCLCPP_DEBUG_STREAM(
    nh_->get_logger(), "[UndoPathGlobalPlanner] received backward path msg poses ["
                         << lastForwardPathMsg_.poses.size() << "]");
}

/**
 ******************************************************************************************************************
 * clearGoalMarker()
 ******************************************************************************************************************
 */
void UndoPathGlobalPlanner::clearGoalMarker()
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
 * publishGoalMarker()
 ******************************************************************************************************************
 */
void UndoPathGlobalPlanner::publishGoalMarker(
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
  marker.scale.x = 0.1;
  marker.scale.y = 0.3;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;

  marker.lifetime = rclcpp::Duration(0s);

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
/**
 ******************************************************************************************************************
 * defaultBackwardPath()
 ******************************************************************************************************************
 */
void UndoPathGlobalPlanner::createDefaultUndoPathPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & /*goal*/,
  std::vector<geometry_msgs::msg::PoseStamped> & plan)
{
  //------------- TRANSFORM TO GLOBAL FRAME PATH ---------------------------
  // the forward plan might be recoreded in a different frame of the global (costmap) frame. Transform it.
  // transform global plan to the navigation reference frame

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] Transforming forward path");
  nav_msgs::msg::Path transformedPlan;
  rclcpp::Duration ttol = rclcpp::Duration::from_seconds(transform_tolerance_);
  for (auto p : lastForwardPathMsg_.poses)
  {
    geometry_msgs::msg::PoseStamped transformedPose;
    p.header.stamp = nh_->now();  // otherwise we can get some time tolerance error
    transformedPose.header.stamp = nh_->now();
    transformedPose.header.frame_id = costmap_ros_->getGlobalFrameID();
    nav_2d_utils::transformPose(tf_, costmap_ros_->getGlobalFrameID(), p, transformedPose, ttol);
    transformedPlan.poses.push_back(transformedPose);
  }

  lastForwardPathMsg_ = transformedPlan;
  //---------------------------------------------------------------------------

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] finding goal closest point");
  int i = lastForwardPathMsg_.poses.size() - 1;
  double linear_mindist = std::numeric_limits<double>::max();
  int mindistindex = -1;
  double startPoseAngle = tf2::getYaw(start.pose.orientation);
  geometry_msgs::msg::Pose startPositionProjected;

  // The goal of this code is finding the most convenient initial path pose.
  // first, find closest linear point to the current robot position
  // we start from the final goal, that is, the beginning of the trajectory
  // (since this was the forward motion from the odom tracker)
  for (auto & p : transformedPlan.poses /*| boost::adaptors::reversed*/)
  {
    geometry_msgs::msg::PoseStamped pose = p;
    pose.header.frame_id = costmap_ros_->getGlobalFrameID();

    double dx = pose.pose.position.x - start.pose.position.x;
    double dy = pose.pose.position.y - start.pose.position.y;

    double dist = sqrt(dx * dx + dy * dy);
    double angleOrientation = tf2::getYaw(pose.pose.orientation);
    double angleError = fabs(angles::shortest_angular_distance(angleOrientation, startPoseAngle));
    if (dist <= linear_mindist)
    {
      mindistindex = i;
      linear_mindist = dist;
      startPositionProjected = pose.pose;

      RCLCPP_DEBUG_STREAM(
        nh_->get_logger(), "[UndoPathGlobalPlanner] initial start point search, NEWBEST_LINEAR= "
                             << i << ". error, linear: " << linear_mindist
                             << ", angular: " << angleError);
    }
    else
    {
      RCLCPP_DEBUG_STREAM(
        nh_->get_logger(), "[UndoPathGlobalPlanner] initial start point search, skipped= "
                             << i << ". best linear error: " << linear_mindist
                             << ". current error, linear: " << dist << " angular: " << angleError);
    }

    i--;
  }

  double const ERROR_DISTANCE_PURE_SPINNING_FACTOR = 1.5;
  // Concept of second pass: now we only consider a pure spinning motion in this point. We want to consume some very
  // close angular targets, (accepting a larger linear minerror of 1.5 besterror. That is, more or less in the same
  // point).

  RCLCPP_DEBUG(nh_->get_logger(), "[UndoPathGlobalPlanner] second angular pass");
  double angularMinDist = std::numeric_limits<double>::max();

  if (mindistindex >= (int)transformedPlan.poses.size())
    mindistindex =
      transformedPlan.poses.size() -
      1;  // workaround, something is making a out of bound exception in poses array access
  {
    if (transformedPlan.poses.size() == 0)
    {
      RCLCPP_WARN_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] Warning possible bug");
    }

    // ------- FULL FORWARD PASS TO FIND THE STARTING POIINT OF THE FORWARD MOTION ------
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] second pass loop");
    for (int i = mindistindex; i >= 0; i--)
    {
      // warning this index, i refers to some inverse interpretation from the previous loop,
      // (last indexes in this path corresponds to the poses closer to our current position)
      RCLCPP_DEBUG_STREAM(
        nh_->get_logger(), "[UndoPathGlobalPlanner] " << i << "/" << transformedPlan.poses.size());
      auto index = (int)transformedPlan.poses.size() - i - 1;
      if (index < 0 || (size_t)index >= transformedPlan.poses.size())
      {
        RCLCPP_WARN_STREAM(
          nh_->get_logger(),
          "[UndoPathGlobalPlanner] this should not happen. Check implementation.");
        break;
      }
      geometry_msgs::msg::PoseStamped pose =
        transformedPlan.poses[transformedPlan.poses.size() - i - 1];

      RCLCPP_DEBUG_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] global frame");
      pose.header.frame_id = costmap_ros_->getGlobalFrameID();

      double dx = pose.pose.position.x - start.pose.position.x;
      double dy = pose.pose.position.y - start.pose.position.y;

      double dist = sqrt(dx * dx + dy * dy);
      if (dist <= linear_mindist * ERROR_DISTANCE_PURE_SPINNING_FACTOR)
      {
        double angleOrientation = tf2::getYaw(pose.pose.orientation);
        double angleError =
          fabs(angles::shortest_angular_distance(angleOrientation, startPoseAngle));
        if (angleError < angularMinDist)
        {
          angularMinDist = angleError;
          mindistindex = i;
          RCLCPP_DEBUG_STREAM(
            nh_->get_logger(),
            "[UndoPathGlobalPlanner] initial start point search (angular update), NEWBEST_ANGULAR= "
              << i << ". error, linear: " << dist << "(" << linear_mindist << ")"
              << ", angular: " << angleError << "(" << angularMinDist << ")");
        }
        else
        {
          RCLCPP_DEBUG_STREAM(
            nh_->get_logger(),
            "[UndoPathGlobalPlanner] initial start point search (angular update), skipped= "
              << i << ". error, linear: " << dist << "(" << linear_mindist << ")"
              << ", angular: " << angleError << "(" << angularMinDist << ")");
        }
      }
      else
      {
        RCLCPP_DEBUG_STREAM(
          nh_->get_logger(),
          "[UndoPathGlobalPlanner] initial start point search (angular update) not in linear "
          "range, skipped= "
            << i << " linear error: " << dist << "(" << linear_mindist << ")");
      }
    }
  }

  // REVERSE FORWARD PASS
  if (mindistindex != -1)
  {
    // plan.push_back(start);

    RCLCPP_WARN_STREAM(
      nh_->get_logger(),
      "[UndoPathGlobalPlanner] Creating the backwards plan from odom tracker path (, "
        << transformedPlan.poses.size() << ") poses");

    RCLCPP_WARN_STREAM(
      nh_->get_logger(), "[UndoPathGlobalPlanner] closer point to goal i="
                           << mindistindex << " (linear min dist " << linear_mindist << ")");

    // copy the path at the inverse direction, but only up to the closest point to the goal in the path  (for partial undoing)
    for (int i = transformedPlan.poses.size() - 1; i >= mindistindex; i--)
    {
      auto & pose = transformedPlan.poses[i];

      rclcpp::Time t(pose.header.stamp);

      RCLCPP_INFO_STREAM(
        nh_->get_logger(),
        "[UndoPathGlobalPlanner] adding to plan i = " << i << " stamp:" << t.seconds());
      plan.push_back(pose);
    }
    RCLCPP_WARN_STREAM(
      nh_->get_logger(), "[UndoPathGlobalPlanner] refined plan has " << plan.size() << "  points");
  }
  else
  {
    RCLCPP_ERROR_STREAM(
      nh_->get_logger(), "[UndoPathGlobalPlanner ] undo global plan size:  " << plan.size());
  }
}

/**
 ******************************************************************************************************************
 * makePlan()
 ******************************************************************************************************************
 */
nav_msgs::msg::Path UndoPathGlobalPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  // -------------- BASIC CHECKS ---------------------

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] Undo global plan start ");
  nav_msgs::msg::Path planMsg;
  std::vector<geometry_msgs::msg::PoseStamped> & plan = planMsg.poses;

  RCLCPP_INFO_STREAM(
    nh_->get_logger(),
    "[UndoPathGlobalPlanner] last forward path msg size: " << lastForwardPathMsg_.poses.size());
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[UndoPathGlobalPlanner] last forward path frame id: "
                         << lastForwardPathMsg_.poses.front().header.frame_id);
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[UndoPathGlobalPlanner] start pose frame id: " << start.header.frame_id);
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[UndoPathGlobalPlanner] goal pose frame id: " << goal.header.frame_id);

  if (lastForwardPathMsg_.poses.size() == 0)
  {
    return planMsg;
  }

  // ---------- INPUTS ACCOMMODATION -------------------
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] Inputs accommodation");
  geometry_msgs::msg::PoseStamped transformedStart, transformedGoal;
  {
    rclcpp::Duration ttol = rclcpp::Duration::from_seconds(transform_tolerance_);

    geometry_msgs::msg::PoseStamped pstart = start;
    pstart.header.stamp = nh_->now();
    nav_2d_utils::transformPose(
      tf_, costmap_ros_->getGlobalFrameID(), pstart, transformedStart, ttol);
    transformedStart.header.frame_id = costmap_ros_->getGlobalFrameID();

    // geometry_msgs::msg::PoseStamped pgoal = goal;
    // pgoal.header.stamp = nh_->now();
    // nav_2d_utils::transformPose(tf_, costmap_ros_->getGlobalFrameID(), pgoal, transformedGoal, ttol);
    // transformedGoal.header.frame_id = costmap_ros_->getGlobalFrameID();

    //--------------- FORCE GOAL POSE----------------------------
    RCLCPP_INFO_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] Forced goal");
    auto forcedGoal =
      lastForwardPathMsg_.poses[lastForwardPathMsg_.poses.size() - 1];  // FORCE LAST POSE
    forcedGoal.header.stamp = nh_->now();
    nav_2d_utils::transformPose(
      tf_, costmap_ros_->getGlobalFrameID(), forcedGoal, transformedGoal, ttol);
    transformedGoal.header.frame_id = costmap_ros_->getGlobalFrameID();
  }

  //------------- CREATING GLOBAL PLAN -----------------------------------------------
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] Creating undo plan");
  this->createDefaultUndoPathPlan(transformedStart, transformedGoal, plan);
  planMsg.header.frame_id = this->costmap_ros_->getGlobalFrameID();

  RCLCPP_INFO_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] publishing goal markers");
  publishGoalMarker(plan.back().pose, 1.0, 0, 1.0 /*purple color*/);

  //--------  CHECKING VALID PLAN ------------------------------------
  bool acceptedGlobalPlan = true;
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[UndoPathGlobalPlanner] valid plan checking");

  auto costmap2d = this->costmap_ros_->getCostmap();
  for (auto & p : plan)
  {
    unsigned int mx, my;
    costmap2d->worldToMap(p.pose.position.x, p.pose.position.y, mx, my);
    auto cost = costmap2d->getCost(mx, my);

    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
    {
      acceptedGlobalPlan = false;
      break;
    }
  }

  //--------  PUBLISHING RESULTS ---------------------------------------
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[UndoPathGlobalPlanner] plan publishing. size: " << plan.size());
  planPub_->publish(planMsg);
  if (!acceptedGlobalPlan)
  {
    RCLCPP_INFO(
      nh_->get_logger(),
      "[UndoPathGlobalPlanner] not accepted global plan because of possible collision");
  }

  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[UndoPathGlobalPlanner] plan publishing. size: " << planMsg.poses.size());

  return planMsg;
}

}  // namespace undo_path_global_planner
}  // namespace cl_nav2z
PLUGINLIB_EXPORT_CLASS(
  cl_nav2z::undo_path_global_planner::UndoPathGlobalPlanner, nav2_core::GlobalPlanner)
