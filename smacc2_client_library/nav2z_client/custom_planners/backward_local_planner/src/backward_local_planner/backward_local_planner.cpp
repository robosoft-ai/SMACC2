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
#include <backward_local_planner/backward_local_planner.hpp>
#include <nav2z_planners_common/common.hpp>

#include <boost/intrusive_ptr.hpp>
#include <chrono>
#include <nav_2d_utils/tf_help.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(
  cl_nav2z::backward_local_planner::BackwardLocalPlanner, nav2_core::Controller)

using namespace std::literals::chrono_literals;

namespace cl_nav2z
{
namespace backward_local_planner
{
/**
 ******************************************************************************************************************
 * BackwardLocalPlanner()
 ******************************************************************************************************************
 */
BackwardLocalPlanner::BackwardLocalPlanner() : waitingTimeout_(0s) {}

/**
 ******************************************************************************************************************
 * ~BackwardLocalPlanner()
 ******************************************************************************************************************
 */
BackwardLocalPlanner::~BackwardLocalPlanner() {}

void BackwardLocalPlanner::activate()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "activating controller BackwardLocalPlanner");
  updateParameters();

  goalMarkerPublisher_->on_activate();
  planPub_->on_activate();
  backwardsPlanPath_.clear();
}

void BackwardLocalPlanner::deactivate()
{
  this->clearMarkers();
  RCLCPP_WARN_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] deactivated");
  planPub_->on_deactivate();
  goalMarkerPublisher_->on_deactivate();
}

void BackwardLocalPlanner::cleanup()
{
  this->clearMarkers();
  RCLCPP_WARN_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] cleanup");
  this->backwardsPlanPath_.clear();
  this->currentCarrotPoseIndex_ = 0;
}

/**
 ******************************************************************************************************************
 * BackwardLocalPlanner::configure()
 ******************************************************************************************************************
 */

template <typename T>
void tryGetOrSet(rclcpp_lifecycle::LifecycleNode::SharedPtr & node, std::string param, T & value)
{
  if (!node->get_parameter(param, value))
  {
    node->set_parameter(rclcpp::Parameter(param, value));
  }
}

void BackwardLocalPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
{
  this->costmapRos_ = costmap_ros;
  // rclcpp::Node::SharedPtr nh("~/BackwardLocalPlanner");
  this->nh_ = parent.lock();
  this->name_ = name;
  this->tf_ = tf;

  k_rho_ = -1.0;
  k_alpha_ = 0.5;
  k_betta_ = -1.0;  // set to zero means that orientation is not important
  carrot_angular_distance_ = 0.4;
  linear_mode_rho_error_threshold_ = 0.02;
  straightBackwardsAndPureSpinningMode_ = true;
  max_linear_x_speed_ = 1.0;
  max_angular_z_speed_ = 2.0;
  yaw_goal_tolerance_ = -1;
  xy_goal_tolerance_ = -1;
  waitingTimeout_ = rclcpp::Duration(10s);

  this->currentCarrotPoseIndex_ = 0;

  declareOrSet(
    nh_, name_ + ".pure_spinning_straight_line_mode", straightBackwardsAndPureSpinningMode_);

  declareOrSet(nh_, name_ + ".k_rho", k_rho_);
  declareOrSet(nh_, name_ + ".k_alpha", k_alpha_);
  declareOrSet(nh_, name_ + ".k_betta", k_betta_);
  declareOrSet(nh_, name_ + ".linear_mode_rho_error_threshold", linear_mode_rho_error_threshold_);

  declareOrSet(nh_, name_ + ".carrot_distance", carrot_distance_);
  declareOrSet(nh_, name_ + ".carrot_angular_distance", carrot_angular_distance_);
  declareOrSet(nh_, name_ + ".enable_obstacle_checking", enable_obstacle_checking_);

  declareOrSet(nh_, name_ + ".max_linear_x_speed", max_linear_x_speed_);
  declareOrSet(nh_, name_ + ".max_angular_z_speed", max_angular_z_speed_);

  // we have to do this, for example for the case we are refining the final orientation.
  // check at some point if the carrot is reached in "goal linear distance", then we go into
  // some automatic pure-spinning mode where we only update the orientation
  // This means that if we reach the carrot with precision we go into pure spinning mode but we cannot
  // leave that point (maybe this could be improved)

  if (yaw_goal_tolerance_ != -1 && carrot_angular_distance_ < yaw_goal_tolerance_)
  {
    RCLCPP_WARN_STREAM(
      nh_->get_logger(), "[BackwardLocalPlanner] carrot_angular_distance ("
                           << carrot_angular_distance_
                           << ") cannot be lower than yaw_goal_tolerance (" << yaw_goal_tolerance_
                           << ") setting carrot_angular_distance = " << yaw_goal_tolerance_);
    carrot_angular_distance_ = yaw_goal_tolerance_;
  }

  if (xy_goal_tolerance_ != -1 && carrot_distance_ < xy_goal_tolerance_)
  {
    RCLCPP_WARN_STREAM(
      nh_->get_logger(), "[BackwardLocalPlanner] carrot_linear_distance ("
                           << carrot_distance_ << ") cannot be lower than xy_goal_tolerance_ ("
                           << yaw_goal_tolerance_
                           << ") setting carrot_angular_distance = " << xy_goal_tolerance_);
    carrot_distance_ = xy_goal_tolerance_;
  }

  goalMarkerPublisher_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "backward_local_planner/goal_marker", 1);

  planPub_ = nh_->create_publisher<nav_msgs::msg::Path>("backward_local_planner/path", 1);
}

void BackwardLocalPlanner::updateParameters()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "--- parameters ---");
  tryGetOrSet(nh_, name_ + ".k_rho", k_rho_);
  RCLCPP_INFO_STREAM(nh_->get_logger(), name_ + ".k_rho:" << k_rho_);
  tryGetOrSet(nh_, name_ + ".k_alpha", k_alpha_);
  RCLCPP_INFO_STREAM(nh_->get_logger(), name_ + ".k_alpha:" << k_alpha_);
  tryGetOrSet(nh_, name_ + ".k_betta", k_betta_);
  RCLCPP_INFO_STREAM(nh_->get_logger(), name_ + ".k_betta:" << k_betta_);

  tryGetOrSet(nh_, name_ + ".enable_obstacle_checking", enable_obstacle_checking_);
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), name_ + ".enable_obstacle_checking: " << enable_obstacle_checking_);

  tryGetOrSet(nh_, name_ + ".carrot_distance", carrot_distance_);
  RCLCPP_INFO_STREAM(nh_->get_logger(), name_ + ".carrot_distance:" << carrot_distance_);
  tryGetOrSet(nh_, name_ + ".carrot_angular_distance", carrot_angular_distance_);
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), name_ + ".carrot_angular_distance: " << carrot_angular_distance_);

  tryGetOrSet(
    nh_, name_ + ".pure_spinning_straight_line_mode", straightBackwardsAndPureSpinningMode_);
  RCLCPP_INFO_STREAM(
    nh_->get_logger(),
    name_ + ".pure_spinning_straight_line_mode: " << straightBackwardsAndPureSpinningMode_);

  tryGetOrSet(nh_, name_ + ".linear_mode_rho_error_threshold", linear_mode_rho_error_threshold_);
  RCLCPP_INFO_STREAM(
    nh_->get_logger(),
    name_ + ".linear_mode_rho_error_threshold: " << linear_mode_rho_error_threshold_);
  tryGetOrSet(nh_, name_ + ".max_linear_x_speed", max_linear_x_speed_);
  RCLCPP_INFO_STREAM(nh_->get_logger(), name_ + ".max_linear_x_speed: " << max_linear_x_speed_);
  tryGetOrSet(nh_, name_ + ".max_angular_z_speed", max_angular_z_speed_);
  RCLCPP_INFO_STREAM(nh_->get_logger(), name_ + ".max_angular_z_speed: " << max_angular_z_speed_);

  if (yaw_goal_tolerance_ != -1 && carrot_angular_distance_ < yaw_goal_tolerance_)
  {
    RCLCPP_WARN_STREAM(
      nh_->get_logger(), "[BackwardLocalPlanner] carrot_angular_distance ("
                           << carrot_angular_distance_
                           << ") cannot be lower than yaw_goal_tolerance (" << yaw_goal_tolerance_
                           << ") setting carrot_angular_distance = " << yaw_goal_tolerance_);
    carrot_angular_distance_ = yaw_goal_tolerance_;
    nh_->set_parameter(
      rclcpp::Parameter(name_ + ".carrot_angular_distance", carrot_angular_distance_));
  }
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), name_ + ".carrot_angular_distance: " << carrot_angular_distance_);

  if (xy_goal_tolerance_ != -1 && carrot_distance_ < xy_goal_tolerance_)
  {
    RCLCPP_WARN_STREAM(
      nh_->get_logger(), "[BackwardLocalPlanner] carrot_linear_distance ("
                           << carrot_distance_ << ") cannot be lower than xy_goal_tolerance_ ("
                           << yaw_goal_tolerance_
                           << ") setting carrot_angular_distance = " << xy_goal_tolerance_);
    carrot_distance_ = xy_goal_tolerance_;
    nh_->set_parameter(rclcpp::Parameter(name_ + ".carrot_distance", carrot_distance_));
  }
  RCLCPP_INFO_STREAM(nh_->get_logger(), name_ + ".carrot_distance:" << carrot_distance_);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "--- end params ---");
}

void BackwardLocalPlanner::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/)
{
  RCLCPP_WARN_STREAM(
    nh_->get_logger(),
    "BackwardLocalPlanner::setSpeedLimit invoked. Ignored, funcionality not "
    "implemented.");
}
/**
 ******************************************************************************************************************
 * auxiliary functions
 ******************************************************************************************************************
 */
void BackwardLocalPlanner::computeCurrentEuclideanAndAngularErrorsToCarrotGoal(
  const geometry_msgs::msg::PoseStamped & tfpose, double & dist, double & angular_error)
{
  double angle = tf2::getYaw(tfpose.pose.orientation);
  auto & carrot_pose = backwardsPlanPath_[currentCarrotPoseIndex_];
  const geometry_msgs::msg::Point & carrot_point = carrot_pose.pose.position;

  tf2::Quaternion carrot_orientation;
  tf2::convert(carrot_pose.pose.orientation, carrot_orientation);
  geometry_msgs::msg::Pose currentPoseDebugMsg = tfpose.pose;

  // take error from the current position to the path point
  double dx = carrot_point.x - tfpose.pose.position.x;
  double dy = carrot_point.y - tfpose.pose.position.y;

  dist = sqrt(dx * dx + dy * dy);

  double pangle = tf2::getYaw(carrot_orientation);
  angular_error = fabs(angles::shortest_angular_distance(pangle, angle));

  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardLocalPlanner] Compute carrot errors from current pose. (linear "
                         << dist << ")(angular " << angular_error << ")" << std::endl
                         << "Current carrot pose: " << std::endl
                         << carrot_pose << std::endl
                         << "Current actual pose:" << std::endl
                         << currentPoseDebugMsg);
}

/**
 ******************************************************************************************************************
 * updateCarrotGoal()
 ******************************************************************************************************************
 */
bool BackwardLocalPlanner::updateCarrotGoal(const geometry_msgs::msg::PoseStamped & tfpose)
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[BackwardsLocalPlanner] --- Carrot update ---");
  double disterr = 0, angleerr = 0;
  // iterate the point from the current position and backward until reaching a new goal point in the path
  // this algorithm among other advantages has that skip the looping with an eager global planner
  // that recalls the same plan (the already performed part of the plan in the current pose is skipped)
  while (currentCarrotPoseIndex_ < (long)backwardsPlanPath_.size() - 1)
  {
    computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, disterr, angleerr);

    RCLCPP_INFO_STREAM(
      nh_->get_logger(), "[BackwardsLocalPlanner] update carrot goal: Current index: "
                           << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size());
    RCLCPP_INFO(
      nh_->get_logger(),
      "[BackwardsLocalPlanner] update carrot goal: linear error %lf, angular error: %lf", disterr,
      angleerr);

    // target pose found, goal carrot tries to escape!
    if (disterr < carrot_distance_ && angleerr < carrot_angular_distance_)
    {
      currentCarrotPoseIndex_++;
      resetDivergenceDetection();
      RCLCPP_INFO_STREAM(
        nh_->get_logger(), "[BackwardsLocalPlanner] move carrot fw "
                             << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size());
    }
    else
    {
      // carrot already escaped
      break;
    }
  }
  // RCLCPP_INFO(nh_->get_logger(),"[BackwardsLocalPlanner] computing angular error");
  if (
    currentCarrotPoseIndex_ >= (long)backwardsPlanPath_.size() - 1 && backwardsPlanPath_.size() > 0)
  {
    currentCarrotPoseIndex_ = backwardsPlanPath_.size() - 1;
    // reupdated errors
    computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, disterr, angleerr);
  }

  RCLCPP_INFO(
    nh_->get_logger(), "[BackwardsLocalPlanner] Current index carrot goal: %d",
    currentCarrotPoseIndex_);
  RCLCPP_INFO(
    nh_->get_logger(),
    "[BackwardsLocalPlanner] Update carrot goal: linear error  %lf (xytol: %lf), angular error: "
    "%lf",
    disterr, xy_goal_tolerance_, angleerr);

  bool carrotInGoalLinearRange = disterr < xy_goal_tolerance_;
  RCLCPP_INFO(
    nh_->get_logger(), "[BackwardsLocalPlanner] carrot in goal radius: %d",
    carrotInGoalLinearRange);

  RCLCPP_INFO(nh_->get_logger(), "[BackwardsLocalPlanner] ---End carrot update---");

  return carrotInGoalLinearRange;
}

bool BackwardLocalPlanner::resetDivergenceDetection()
{
  // this function should be called always the carrot is updated
  divergenceDetectionLastCarrotLinearDistance_ = std::numeric_limits<double>::max();
  return true;
}

bool BackwardLocalPlanner::divergenceDetectionUpdate(const geometry_msgs::msg::PoseStamped & tfpose)
{
  double disterr = 0, angleerr = 0;
  computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, disterr, angleerr);

  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardLocalPlanner] Divergence check. carrot goal distance. was: "
                         << divergenceDetectionLastCarrotLinearDistance_
                         << ", now it is: " << disterr);
  if (disterr > divergenceDetectionLastCarrotLinearDistance_)
  {
    // candidate of divergence, we do not throw the divergence alarm yet
    // but we neither update the distance since it is worse than the one
    // we had previously with the same carrot.
    const double MARGIN_FACTOR = 1.2;
    if (disterr > MARGIN_FACTOR * divergenceDetectionLastCarrotLinearDistance_)
    {
      RCLCPP_ERROR_STREAM(
        nh_->get_logger(),
        "[BackwardLocalPlanner] Divergence detected. The same carrot goal distance was previously: "
          << divergenceDetectionLastCarrotLinearDistance_ << "but now it is: " << disterr);
      return true;
    }
    else
    {
      // divergence candidate
      return false;
    }
  }
  else
  {
    // update:
    divergenceDetectionLastCarrotLinearDistance_ = disterr;
    return false;
  }
}

bool BackwardLocalPlanner::checkCarrotHalfPlainConstraint(
  const geometry_msgs::msg::PoseStamped & tfpose)
{
  // this function is specially useful when we want to reach the goal with a lot
  // of precision. We may pass the goal and then the controller enters in some
  // unstable state. With this, we are able to detect when stop moving.

  // only apply if the carrot is in goal position and also if we are not in a pure spinning behavior v!=0

  auto & carrot_pose = backwardsPlanPath_[currentCarrotPoseIndex_];
  const geometry_msgs::msg::Point & carrot_point = carrot_pose.pose.position;
  double yaw = tf2::getYaw(carrot_pose.pose.orientation);

  // direction vector
  double vx = cos(yaw);
  double vy = sin(yaw);

  // line implicit equation
  // ax + by + c = 0
  double c = -vx * carrot_point.x - vy * carrot_point.y;
  const double C_OFFSET_METERS = 0.05;  // 5 cm
  double check = vx * tfpose.pose.position.x + vy * tfpose.pose.position.y + c + C_OFFSET_METERS;

  RCLCPP_INFO_STREAM(
    nh_->get_logger(),
    "[BackwardLocalPlanner] half plane constraint:" << vx << "*" << carrot_point.x << " + " << vy
                                                    << "*" << carrot_point.y << " + " << c);
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardLocalPlanner] constraint evaluation: "
                         << vx << "*" << tfpose.pose.position.x << " + " << vy << "*"
                         << tfpose.pose.position.y << " + " << c << " = " << check);

  return check < 0;
}

bool BackwardLocalPlanner::checkCurrentPoseInGoalRange(
  const geometry_msgs::msg::PoseStamped & tfpose,
  const geometry_msgs::msg::Twist & /*currentTwist*/, double angle_error, bool & linearGoalReached,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  auto & finalgoal = backwardsPlanPath_.back();
  double gdx = finalgoal.pose.position.x - tfpose.pose.position.x;
  double gdy = finalgoal.pose.position.y - tfpose.pose.position.y;
  double goaldist = sqrt(gdx * gdx + gdy * gdy);

  auto abs_angle_error = fabs(angle_error);
  // final_alpha_error =
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardLocalPlanner] goal check. linear dist: "
                         << goaldist << "(" << this->xy_goal_tolerance_ << ")"
                         << ", angular dist: " << abs_angle_error << "("
                         << this->yaw_goal_tolerance_ << ")");

  linearGoalReached = goaldist < this->xy_goal_tolerance_;

  return linearGoalReached && abs_angle_error < this->yaw_goal_tolerance_;
  // return goal_checker->isGoalReached(tfpose.pose, finalgoal.pose, currentTwist);
}

/**
 ******************************************************************************************************************
 * pureSpinningCmd()
 ******************************************************************************************************************
 */
void BackwardLocalPlanner::straightBackwardsAndPureSpinCmd(
  const geometry_msgs::msg::PoseStamped & /*tfpose*/, double & vetta, double & gamma,
  double alpha_error, double betta_error, double rho_error)
{
  if (rho_error > linear_mode_rho_error_threshold_)  // works in straight motion mode
  {
    vetta = k_rho_ * rho_error;
    gamma = k_alpha_ * alpha_error;
  }
  else if (fabs(betta_error) >= this->yaw_goal_tolerance_)  // works in pure spinning mode
  {
    vetta = 0;  // disable linear
    gamma = k_betta_ * betta_error;
  }
}

/**
 ******************************************************************************************************************
 * computeVelocityCommands()
 ******************************************************************************************************************
 */
geometry_msgs::msg::TwistStamped BackwardLocalPlanner::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  RCLCPP_INFO(
    nh_->get_logger(),
    "[BackwardLocalPlanner] ------------------- LOCAL PLANNER LOOP -----------------");
  this->updateParameters();

  // consistency check
  if (this->backwardsPlanPath_.size() > 0)
  {
    RCLCPP_INFO_STREAM(
      nh_->get_logger(), "[BackwardLocalPlanner] Current pose frame id: "
                           << backwardsPlanPath_.front().header.frame_id
                           << ", path pose frame id: " << pose.header.frame_id);

    if (backwardsPlanPath_.front().header.frame_id != pose.header.frame_id)
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] Inconsistent frames");
    }
  }

  // xy_goal_tolerance and yaw_goal_tolerance are just used for logging proposes and clamping the carrot
  // goal distance (parameter safety)
  if (xy_goal_tolerance_ == -1 || yaw_goal_tolerance_ == -1)
  {
    geometry_msgs::msg::Pose posetol;
    geometry_msgs::msg::Twist twistol;
    if (goal_checker->getTolerances(posetol, twistol))
    {
      xy_goal_tolerance_ = posetol.position.x;
      yaw_goal_tolerance_ = tf2::getYaw(posetol.orientation);
      //xy_goal_tolerance_ = posetol.position.x * 0.35;  // WORKAROUND ISSUE GOAL CHECKER NAV_CONTROLLER DIFF
      //yaw_goal_tolerance_ = tf2::getYaw(posetol.orientation) * 0.35;
      RCLCPP_INFO_STREAM(
        nh_->get_logger(), "[BackwardLocalPlanner] xy_goal_tolerance_: "
                             << xy_goal_tolerance_
                             << ", yaw_goal_tolerance_: " << yaw_goal_tolerance_);
    }
    else
    {
      RCLCPP_INFO_STREAM(
        nh_->get_logger(), "[BackwardLocalPlanner] could not get tolerances from goal checker");
    }
  }

  RCLCPP_INFO(
    nh_->get_logger(),
    "[BackwardLocalPlanner] ------------------- LOCAL PLANNER LOOP -----------------");

  geometry_msgs::msg::TwistStamped cmd_vel;
  RCLCPP_INFO(nh_->get_logger(), "[BackwardLocalPlanner] LOCAL PLANNER LOOP");
  geometry_msgs::msg::PoseStamped paux;
  geometry_msgs::msg::PoseStamped tfpose;

  if (!costmapRos_->getRobotPose(tfpose))
  {
    RCLCPP_ERROR(
      nh_->get_logger(),
      "[BackwardLocalPlanner] missing robot pose, canceling compute Velocity Command");
  }  // it is not working in the pure spinning reel example, maybe the hyperplane check is enough
  bool divergenceDetected = false;

  bool emergency_stop = false;
  if (divergenceDetected)
  {
    RCLCPP_ERROR(
      nh_->get_logger(), "[BackwardLocalPlanner] Divergence detected. Sending emergency stop.");
    emergency_stop = true;
  }

  bool carrotInLinearGoalRange = updateCarrotGoal(tfpose);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] carrot goal created");

  if (emergency_stop)
  {
    cmd_vel.twist.linear.x = 0;
    cmd_vel.twist.angular.z = 0;
    RCLCPP_INFO_STREAM(
      nh_->get_logger(), "[BackwardLocalPlanner] emergency stop, exit compute commands");
    // return false;
    return cmd_vel;
  }

  // ------ Evaluate the current context ----
  double rho_error, betta_error, alpha_error;

  // getting carrot goal information
  tf2::Quaternion q;
  tf2::convert(tfpose.pose.orientation, q);

  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardLocalPlanner] carrot goal: " << currentCarrotPoseIndex_ << "/"
                                                              << backwardsPlanPath_.size());
  const geometry_msgs::msg::PoseStamped & carrotgoalpose =
    backwardsPlanPath_[currentCarrotPoseIndex_];
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardLocalPlanner] carrot goal pose current index: "
                         << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size() << ": "
                         << carrotgoalpose);
  const geometry_msgs::msg::Point & carrotGoalPosition = carrotgoalpose.pose.position;

  tf2::Quaternion goalQ;
  tf2::fromMsg(carrotgoalpose.pose.orientation, goalQ);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] -- Control Policy --");
  // goal orientation (global frame)
  double betta = tf2::getYaw(goalQ);
  RCLCPP_INFO_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] goal orientation: " << betta);
  betta = betta + betta_offset_;

  double dx = carrotGoalPosition.x - tfpose.pose.position.x;
  double dy = carrotGoalPosition.y - tfpose.pose.position.y;

  // distance error to the targetpoint
  rho_error = sqrt(dx * dx + dy * dy);

  // heading to goal angle
  double theta = tf2::getYaw(q);
  double alpha = atan2(dy, dx);
  alpha = alpha + alpha_offset_;

  alpha_error = angles::shortest_angular_distance(alpha, theta);
  betta_error = angles::shortest_angular_distance(betta, theta);
  //------------- END CONTEXT EVAL ----------

  bool linearGoalReached;
  bool currentPoseInGoal =
    checkCurrentPoseInGoalRange(tfpose, velocity, betta_error, linearGoalReached, goal_checker);

  // Make sure the robot is very close to the goal and it is really in the the last goal point.
  bool carrotInFinalGoalIndex = currentCarrotPoseIndex_ == (int)backwardsPlanPath_.size() - 1;

  // checking if we are really in the end goal pose
  if (currentPoseInGoal && carrotInFinalGoalIndex)
  {
    goalReached_ = true;
    // backwardsPlanPath_.clear();
    RCLCPP_INFO_STREAM(
      nh_->get_logger(),
      "[BackwardLocalPlanner] GOAL REACHED. Send stop command and skipping trajectory collision: "
        << cmd_vel.twist);
    cmd_vel.twist.linear.x = 0;
    cmd_vel.twist.angular.z = 0;
    return cmd_vel;
  }
  else if (
    carrotInLinearGoalRange &&
    linearGoalReached)  // checking if we are in the end goal point but with incorrect
                        // orientation
  {
    // this means that we are not in the final angular distance, and we may even not be in the last carrot index
    // (several intermediate angular poses until the last goal pose)
    inGoalPureSpinningState_ = true;
  }

  // --------------------
  double vetta, gamma;
  if (straightBackwardsAndPureSpinningMode_)
  {
    // decorated control rule for this mode
    this->straightBackwardsAndPureSpinCmd(
      tfpose, vetta, gamma, alpha_error, betta_error, rho_error);
  }
  else  // default free navigation backward motion mode
  {
    // regular control rule
    vetta = k_rho_ * rho_error;
    gamma = k_alpha_ * alpha_error + k_betta_ * betta_error;

    // Even if we are in free navigation, we can enter in the pure spinning state.
    // then, the linear motion is deactivated.
    if (inGoalPureSpinningState_)
    {
      RCLCPP_INFO(
        nh_->get_logger(),
        "[BackwardLocalPlanner] we entered in a pure spinning state even in not pure-spining "
        "configuration, "
        "carrotDistanceGoalReached: %d",
        carrotInLinearGoalRange);
      gamma = k_betta_ * betta_error;
      vetta = 0;
    }

    // classical control to reach a goal backwards
  }

  // Apply command and Clamp to limits
  cmd_vel.twist.linear.x = vetta;
  cmd_vel.twist.angular.z = gamma;

  if (cmd_vel.twist.linear.x > max_linear_x_speed_)
  {
    cmd_vel.twist.linear.x = max_linear_x_speed_;
  }
  else if (cmd_vel.twist.linear.x < -max_linear_x_speed_)
  {
    cmd_vel.twist.linear.x = -max_linear_x_speed_;
  }

  if (cmd_vel.twist.angular.z > max_angular_z_speed_)
  {
    cmd_vel.twist.angular.z = max_angular_z_speed_;
  }
  else if (cmd_vel.twist.angular.z < -max_angular_z_speed_)
  {
    cmd_vel.twist.angular.z = -max_angular_z_speed_;
  }

  publishGoalMarker(carrotGoalPosition.x, carrotGoalPosition.y, betta);

  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardLocalPlanner] local planner,"
                         << std::endl
                         << " current pose in goal: " << currentPoseInGoal << std::endl
                         << " carrot in final goal index: " << carrotInFinalGoalIndex << std::endl
                         << " carrot in linear goal range: " << carrotInLinearGoalRange << std::endl
                         << " straightAnPureSpiningMode: " << straightBackwardsAndPureSpinningMode_
                         << std::endl
                         << " inGoalPureSpinningState: " << inGoalPureSpinningState_ << std::endl
                         << " theta: " << theta << std::endl
                         << " betta: " << theta << std::endl
                         << " err_x: " << dx << std::endl
                         << " err_y:" << dy << std::endl
                         << " rho_error:" << rho_error << std::endl
                         << " alpha_error:" << alpha_error << std::endl
                         << " betta_error:" << betta_error << std::endl
                         << " vetta:" << vetta << std::endl
                         << " gamma:" << gamma << std::endl
                         << " cmd_vel.lin.x:" << cmd_vel.twist.linear.x << std::endl
                         << " cmd_vel.ang.z:" << cmd_vel.twist.angular.z);

  if (inGoalPureSpinningState_)
  {
    bool carrotHalfPlaneConstraintFailure = checkCarrotHalfPlainConstraint(tfpose);

    if (carrotHalfPlaneConstraintFailure)
    {
      RCLCPP_ERROR(
        nh_->get_logger(),
        "[BackwardLocalPlanner] CarrotHalfPlaneConstraintFailure detected. Sending "
        "emergency stop and success to the planner.");
      cmd_vel.twist.linear.x = 0;
    }
  }

  // ---------------------- TRAJECTORY PREDICTION AND COLLISION AVOIDANCE ---------------------
  // cmd_vel.twist.linear.x=0;
  // cmd_vel.twist.angular.z = 0;

  geometry_msgs::msg::PoseStamped global_pose;
  costmapRos_->getRobotPose(global_pose);

  auto * costmap2d = costmapRos_->getCostmap();
  auto yaw = tf2::getYaw(global_pose.pose.orientation);

  auto & pos = global_pose.pose.position;

  Eigen::Vector3f currentpose(pos.x, pos.y, yaw);
  Eigen::Vector3f currentvel(
    cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);
  std::vector<Eigen::Vector3f> trajectory;
  this->generateTrajectory(
    currentpose, currentvel, 0.8 /*meters*/, M_PI / 8 /*rads*/, 3.0 /*seconds*/, 0.05 /*seconds*/,
    trajectory);

  // check plan rejection
  bool acceptedLocalTrajectoryFreeOfObstacles = true;

  unsigned int mx, my;

  if (this->enable_obstacle_checking_)
  {
    if (backwardsPlanPath_.size() > 0)
    {
      auto & finalgoalpose = backwardsPlanPath_.back();

      int i = 0;
      // RCLCPP_INFO_STREAM(nh_->get_logger(), "lplanner goal: " << finalgoalpose.pose.position);
      geometry_msgs::msg::Twist mockzerospeed;

      for (auto & p : trajectory)
      {
        /*geometry_msgs::msg::Pose pg;
        pg.position.x = p[0];
        pg.position.y = p[1];
        tf2::Quaternion q;
        q.setRPY(0, 0, p[2]);
        pg.orientation = tf2::toMsg(q);

        // WARNING I CAN'T USE isGoalReached because I can change the state of a stateful goal checker
        if (goal_checker->isGoalReached(pg, finalgoalpose.pose, mockzerospeed))*/

        float dx = p[0] - finalgoalpose.pose.position.x;
        float dy = p[1] - finalgoalpose.pose.position.y;

        float dst = sqrt(dx * dx + dy * dy);
        if (dst < xy_goal_tolerance_)
        {
          RCLCPP_INFO(
            nh_->get_logger(),
            "[BackwardLocalPlanner] trajectory simulation for collision checking: goal "
            "reached with no collision");
          break;
        }

        costmap2d->worldToMap(p[0], p[1], mx, my);
        //         unsigned int cost = costmap2d->getCost(mx, my);

        // RCLCPP_INFO(nh_->get_logger(),"[BackwardLocalPlanner] checking cost pt %d [%lf, %lf] cell[%d,%d] = %d", i,
        // p[0], p[1], mx, my, cost); RCLCPP_INFO_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] cost: " << cost);

        // static const unsigned char NO_INFORMATION = 255;
        // static const unsigned char LETHAL_OBSTACLE = 254;
        // static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
        // static const unsigned char FREE_SPACE = 0;

        if (costmap2d->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {
          acceptedLocalTrajectoryFreeOfObstacles = false;
          RCLCPP_WARN_STREAM(
            nh_->get_logger(),
            "[BackwardLocalPlanner] ABORTED LOCAL PLAN BECAUSE OBSTACLE DETEDTED at point "
              << i << "/" << trajectory.size() << std::endl
              << p[0] << ", " << p[1]);
          break;
        }
        i++;
      }
    }
    else
    {
      RCLCPP_WARN(
        nh_->get_logger(), "[BackwardLocalPlanner] Abort local - Backwards global plan size: %ld",
        backwardsPlanPath_.size());
      cmd_vel.twist.angular.z = 0;
      cmd_vel.twist.linear.x = 0;
      // return false;
    }
  }

  if (acceptedLocalTrajectoryFreeOfObstacles)
  {
    waiting_ = false;
    RCLCPP_INFO(
      nh_->get_logger(),
      "[BackwardLocalPlanner] accepted local trajectory free of obstacle. Local planner "
      "continues.");
    return cmd_vel;
    // return true;
  }
  else  // that is not appceted because existence of obstacles
  {
    // emergency stop for collision: waiting a while before sending error
    cmd_vel.twist.linear.x = 0;
    cmd_vel.twist.angular.z = 0;

    if (waiting_ == false)
    {
      waiting_ = true;
      waitingStamp_ = nh_->now();
      RCLCPP_WARN(
        nh_->get_logger(), "[BackwardLocalPlanner][Not accepted local plan] starting countdown");
    }
    else
    {
      auto waitingduration = nh_->now() - waitingStamp_;

      if (waitingduration > this->waitingTimeout_)
      {
        RCLCPP_WARN(
          nh_->get_logger(), "[BackwardLocalPlanner][Abort local] timeout! duration %lf/%f",
          waitingduration.seconds(), waitingTimeout_.seconds());
        // return false;
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.angular.z = 0;
        return cmd_vel;
      }
    }

    return cmd_vel;
  }
}

/**
 ******************************************************************************************************************
 * isGoalReached()
 ******************************************************************************************************************
 */
bool BackwardLocalPlanner::isGoalReached()
{
  RCLCPP_INFO(nh_->get_logger(), "[BackwardLocalPlanner] isGoalReached call");
  return goalReached_;
}

bool BackwardLocalPlanner::findInitialCarrotGoal(geometry_msgs::msg::PoseStamped & tfpose)
{
  double lineardisterr, angleerr;
  bool inCarrotRange = false;

  // initial state check
  computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, lineardisterr, angleerr);

  // double minpointdist = std::numeric_limits<double>::max();

  // lets set the carrot-goal in the correct place with this loop
  while (currentCarrotPoseIndex_ < (int)backwardsPlanPath_.size() && !inCarrotRange)
  {
    computeCurrentEuclideanAndAngularErrorsToCarrotGoal(tfpose, lineardisterr, angleerr);

    RCLCPP_INFO(
      nh_->get_logger(),
      "[BackwardLocalPlanner] Finding initial carrot goal i=%d - error to carrot, linear = %lf "
      "(%lf), "
      "angular : %lf (%lf)",
      currentCarrotPoseIndex_, lineardisterr, carrot_distance_, angleerr, carrot_angular_distance_);

    // current path point is inside the carrot distance range, goal carrot tries to escape!
    if (lineardisterr < carrot_distance_ && angleerr < carrot_angular_distance_)
    {
      RCLCPP_INFO(
        nh_->get_logger(),
        "[BackwardLocalPlanner] Finding initial carrot goal i=%d - in carrot Range",
        currentCarrotPoseIndex_);
      inCarrotRange = true;
      // we are inside the goal range
    }
    else if (
      inCarrotRange && (lineardisterr > carrot_distance_ || angleerr > carrot_angular_distance_))
    {
      // we were inside the carrot range but not anymore, now we are just leaving. we want to continue forward
      // (currentCarrotPoseIndex_++) unless we go out of the carrot range

      // but we rollback last index increment (to go back inside the carrot goal scope) and start motion with that
      // carrot goal we found
      currentCarrotPoseIndex_--;
      break;
    }
    else
    {
      RCLCPP_INFO(
        nh_->get_logger(),
        "[BackwardLocalPlanner] Finding initial carrot goal i=%d - carrot out of range, searching "
        "coincidence...",
        currentCarrotPoseIndex_);
    }

    currentCarrotPoseIndex_++;
    RCLCPP_INFO_STREAM(
      nh_->get_logger(), "[BackwardLocalPlanner] setPlan: fw" << currentCarrotPoseIndex_);
  }

  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardLocalPlanner] setPlan: (found first carrot:"
                         << inCarrotRange << ") initial carrot point index: "
                         << currentCarrotPoseIndex_ << "/" << backwardsPlanPath_.size());

  return inCarrotRange;
}

bool BackwardLocalPlanner::resamplePrecisePlan()
{
  // this algorithm is really important to have a precise carrot (linear or angular)
  // and not being considered as a divergence from the path

  RCLCPP_INFO(nh_->get_logger(), "[BackwardLocalPlanner] resample precise");
  if (backwardsPlanPath_.size() <= 1)
  {
    RCLCPP_INFO_STREAM(
      nh_->get_logger(),
      "[BackwardLocalPlanner] resample precise skipping, size: " << backwardsPlanPath_.size());
    return false;
  }

  int counter = 0;
  double maxallowedAngularError = 0.45 * this->carrot_angular_distance_;  // nyquist
  double maxallowedLinearError = 0.45 * this->carrot_distance_;           // nyquist

  for (int i = 0; i < (int)backwardsPlanPath_.size() - 1; i++)
  {
    RCLCPP_INFO_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] resample precise, check: " << i);
    auto & currpose = backwardsPlanPath_[i];
    auto & nextpose = backwardsPlanPath_[i + 1];

    tf2::Quaternion qCurrent, qNext;
    tf2::convert(currpose.pose.orientation, qCurrent);
    tf2::convert(nextpose.pose.orientation, qNext);

    double dx = nextpose.pose.position.x - currpose.pose.position.x;
    double dy = nextpose.pose.position.y - currpose.pose.position.y;
    double dist = sqrt(dx * dx + dy * dy);

    bool resample = false;
    if (dist > maxallowedLinearError)
    {
      RCLCPP_INFO_STREAM(
        nh_->get_logger(), "[BackwardLocalPlanner] resampling point, linear distance:"
                             << dist << "(" << maxallowedLinearError << ")" << i);
      resample = true;
    }
    else
    {
      double currentAngle = tf2::getYaw(qCurrent);
      double nextAngle = tf2::getYaw(qNext);

      double angularError = fabs(angles::shortest_angular_distance(currentAngle, nextAngle));
      if (angularError > maxallowedAngularError)
      {
        resample = true;
        RCLCPP_INFO_STREAM(
          nh_->get_logger(), "[BackwardLocalPlanner] resampling point, angular distance:"
                               << angularError << "(" << maxallowedAngularError << ")" << i);
      }
    }

    if (resample)
    {
      geometry_msgs::msg::PoseStamped pintermediate;
      auto duration = rclcpp::Time(nextpose.header.stamp) - rclcpp::Time(currpose.header.stamp);

      pintermediate.header.frame_id = currpose.header.frame_id;
      pintermediate.header.stamp = rclcpp::Time(currpose.header.stamp) + duration * 0.5;

      pintermediate.pose.position.x = 0.5 * (currpose.pose.position.x + nextpose.pose.position.x);
      pintermediate.pose.position.y = 0.5 * (currpose.pose.position.y + nextpose.pose.position.y);
      pintermediate.pose.position.z = 0.5 * (currpose.pose.position.z + nextpose.pose.position.z);
      tf2::Quaternion intermediateQuat = tf2::slerp(qCurrent, qNext, 0.5);
      pintermediate.pose.orientation = tf2::toMsg(intermediateQuat);

      this->backwardsPlanPath_.insert(this->backwardsPlanPath_.begin() + i + 1, pintermediate);

      // retry this point
      i--;
      counter++;
    }
  }

  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "[BackwardLocalPlanner] End resampling. resampled:" << counter
                                                                           << " new inserted poses "
                                                                              "during precise "
                                                                              "resmapling.");
  return true;
}

/**
 ******************************************************************************************************************
 * setPlan()
 ******************************************************************************************************************
 */
void BackwardLocalPlanner::setPlan(const nav_msgs::msg::Path & path)
{
  RCLCPP_INFO_STREAM(
    nh_->get_logger(),
    "[BackwardLocalPlanner] setPlan: new global plan received ( " << path.poses.size() << ")");

  //------------- TRANSFORM TO LOCAL FRAME PATH ---------------------------
  nav_msgs::msg::Path transformedPlan;
  rclcpp::Duration ttol = rclcpp::Duration::from_seconds(transform_tolerance_);
  // transform global plan to the navigation reference frame
  for (auto & p : path.poses)
  {
    geometry_msgs::msg::PoseStamped transformedPose;
    nav_2d_utils::transformPose(tf_, costmapRos_->getGlobalFrameID(), p, transformedPose, ttol);
    transformedPose.header.frame_id = costmapRos_->getGlobalFrameID();
    transformedPlan.poses.push_back(transformedPose);
  }

  backwardsPlanPath_ = transformedPlan.poses;

  // --------- resampling path feature -----------
  geometry_msgs::msg::PoseStamped tfpose;
  if (!costmapRos_->getRobotPose(tfpose))
  {
    RCLCPP_ERROR(nh_->get_logger(), "Failure getting pose from Backward local planner");
    return;
  }

  geometry_msgs::msg::PoseStamped posestamped = tfpose;
  backwardsPlanPath_.insert(backwardsPlanPath_.begin(), posestamped);
  this->resamplePrecisePlan();

  nav_msgs::msg::Path planMsg;
  planMsg.poses = backwardsPlanPath_;
  planMsg.header.frame_id = costmapRos_->getGlobalFrameID();
  planMsg.header.stamp = nh_->now();
  planPub_->publish(planMsg);

  // ------ reset controller state ----------------------
  goalReached_ = false;
  inGoalPureSpinningState_ = false;
  currentCarrotPoseIndex_ = 0;
  this->resetDivergenceDetection();

  if (path.poses.size() == 0)
  {
    RCLCPP_INFO_STREAM(nh_->get_logger(), "[BackwardLocalPlanner] received plan without any pose");
    // return true;
    return;
  }

  // -------- initialize carrot ----------------
  bool foundInitialCarrotGoal = this->findInitialCarrotGoal(tfpose);
  if (!foundInitialCarrotGoal)
  {
    RCLCPP_ERROR(
      nh_->get_logger(),
      "[BackwardLocalPlanner] new plan rejected. The initial point in the global path is "
      "too much far away from the current state (according to carrot_distance "
      "parameter)");
    // return false; // in this case, the new plan broke the current execution
    return;
  }
  else
  {
    this->divergenceDetectionUpdate(tfpose);
    // SANDARD AND PREFERRED CASE ON NEW PLAN
    // return true;
    return;
  }
}

void BackwardLocalPlanner::generateTrajectory(
  const Eigen::Vector3f & pos, const Eigen::Vector3f & vel, float maxdist, float maxanglediff,
  float maxtime, float dt, std::vector<Eigen::Vector3f> & outtraj)
{
  // simulate the trajectory and check for collisions, updating costs along the way
  bool end = false;
  float time = 0;
  Eigen::Vector3f currentpos = pos;
  int i = 0;
  while (!end)
  {
    // add the point to the trajectory so we can draw it later if we want
    // traj.addPoint(pos[0], pos[1], pos[2]);

    // if (continued_acceleration_) {
    //   //calculate velocities
    //   loop_vel = computeNewVelocities(sample_target_vel, loop_vel, limits_->getAccLimits(), dt);
    //   //RCLCPP_WARN_NAMED(nh_->get_logger(), "Generator", "Flag: %d, Loop_Vel %f, %f, %f", continued_acceleration_,
    //   loop_vel[0], loop_vel[1], loop_vel[2]);
    // }

    auto loop_vel = vel;
    // update the position of the robot using the velocities passed in
    auto newpos = computeNewPositions(currentpos, loop_vel, dt);

    auto dx = newpos[0] - currentpos[0];
    auto dy = newpos[1] - currentpos[1];
    float dist, angledist;

    // RCLCPP_INFO(nh_->get_logger(), "traj point %d", i);
    dist = sqrt(dx * dx + dy * dy);
    if (dist > maxdist)
    {
      end = true;
      // RCLCPP_INFO(nh_->get_logger(), "dist break: %f", dist);
    }
    else
    {
      // ouble from, double to
      angledist = angles::shortest_angular_distance(currentpos[2], newpos[2]);
      if (angledist > maxanglediff)
      {
        end = true;
        // RCLCPP_INFO(nh_->get_logger(), "angle dist break: %f", angledist);
      }
      else
      {
        outtraj.push_back(newpos);

        time += dt;
        if (time > maxtime)
        {
          end = true;
          // RCLCPP_INFO(nh_->get_logger(), "time break: %f", time);
        }

        // RCLCPP_INFO(nh_->get_logger(), "dist: %f, angledist: %f, time: %f", dist, angledist, time);
      }
    }

    currentpos = newpos;
    i++;
  }  // end for simulation steps
}

Eigen::Vector3f BackwardLocalPlanner::computeNewPositions(
  const Eigen::Vector3f & pos, const Eigen::Vector3f & vel, double dt)
{
  Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
  new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * dt;
  new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * dt;
  new_pos[2] = pos[2] + vel[2] * dt;
  return new_pos;
}

void BackwardLocalPlanner::clearMarkers()
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = this->costmapRos_->getGlobalFrameID();
  marker.header.stamp = nh_->now();

  marker.ns = "my_namespace2";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;

  visualization_msgs::msg::MarkerArray ma;
  ma.markers.push_back(marker);

  goalMarkerPublisher_->publish(ma);
}

/**
 ******************************************************************************************************************
 * publishGoalMarker()
 ******************************************************************************************************************
 */
void BackwardLocalPlanner::publishGoalMarker(double x, double y, double phi)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = this->costmapRos_->getGlobalFrameID();
  marker.header.stamp = nh_->now();

  marker.ns = "my_namespace2";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(1.0s);

  marker.pose.orientation.w = 1;

  marker.scale.x = 0.05;
  marker.scale.y = 0.15;
  marker.scale.z = 0.05;
  marker.color.a = 1.0;

  // red marker
  marker.color.r = 1;
  marker.color.g = 0;
  marker.color.b = 0;

  geometry_msgs::msg::Point start, end;
  start.x = x;
  start.y = y;

  end.x = x + 0.5 * cos(phi);
  end.y = y + 0.5 * sin(phi);

  marker.points.push_back(start);
  marker.points.push_back(end);

  visualization_msgs::msg::MarkerArray ma;
  ma.markers.push_back(marker);

  goalMarkerPublisher_->publish(ma);
}
}  // namespace backward_local_planner
}  // namespace cl_nav2z
