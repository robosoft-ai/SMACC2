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
#include <nav2z_client/common.hpp>

#include <angles/angles.h>
#include <tf2/utils.h>
#include <nav2z_client/components/odom_tracker/odom_tracker.hpp>

#include <boost/range/adaptor/reversed.hpp>

namespace cl_nav2z
{
namespace odom_tracker
{
OdomTracker::OdomTracker(std::string odomTopicName, std::string odomFrame)
{
  workingMode_ = WorkingMode::RECORD_PATH;
  publishMessages = true;
  subscribeToOdometryTopic_ = true;
  odomFrame_ = odomFrame;
  odomTopicName_ = odomTopicName;
}

template <typename T>
void parameterDeclareAndtryGetOrSet(
  rclcpp::Node::SharedPtr & node, std::string param_name, T & value)
{
  if (!node->get_parameter(param_name, value))
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "[OdomTracker] autoset " << param_name << ": " << value);
    node->declare_parameter(param_name, value);
  }
  else
  {
    RCLCPP_INFO_STREAM(node->get_logger(), "[OdomTracker] " << param_name << ": " << value);
  }
}

void OdomTracker::onInitialize()
{
  // default values
  recordPointDistanceThreshold_ = 0.005;  // 5 mm
  recordAngularDistanceThreshold_ = 0.1;  // radians
  clearPointDistanceThreshold_ = 0.05;    // 5 cm
  clearAngularDistanceThreshold_ = 0.1;   // radi

  auto nh = getNode();
  RCLCPP_WARN(getLogger(), "[OdomTracker] Initializing Odometry Tracker");

  parameterDeclareAndtryGetOrSet(nh, "odom_frame", this->odomFrame_);
  parameterDeclareAndtryGetOrSet(
    nh, "record_point_distance_threshold", recordPointDistanceThreshold_);
  parameterDeclareAndtryGetOrSet(
    nh, "record_angular_distance_threshold", recordAngularDistanceThreshold_);
  parameterDeclareAndtryGetOrSet(
    nh, "clear_point_distance_threshold", clearPointDistanceThreshold_);
  parameterDeclareAndtryGetOrSet(
    nh, "clear_angular_distance_threshold", clearAngularDistanceThreshold_);

  if (this->subscribeToOdometryTopic_)
  {
    rclcpp::SensorDataQoS qos;
    odomSub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
      odomTopicName_, qos,
      std::bind(&OdomTracker::processOdometryMessage, this, std::placeholders::_1));
  }

  robotBasePathPub_ = nh->create_publisher<nav_msgs::msg::Path>("odom_tracker_path", 1);
  robotBasePathStackedPub_ =
    nh->create_publisher<nav_msgs::msg::Path>("odom_tracker_stacked_path", 1);
}

/**
 ******************************************************************************************************************
 * setWorkingMode()
 ******************************************************************************************************************
 */
void OdomTracker::setWorkingMode(WorkingMode workingMode)
{
  // RCLCPP_INFO(getLogger(),"odom_tracker m_mutex acquire");
  std::lock_guard<std::mutex> lock(m_mutex_);

  switch (workingMode)
  {
    case WorkingMode::RECORD_PATH:
      RCLCPP_INFO_STREAM(
        getLogger(),
        "[OdomTracker] setting working mode to RECORD - record_point_distance_threshold: "
          << recordPointDistanceThreshold_
          << ", record_angular_distance_threshold: " << recordAngularDistanceThreshold_);
      break;
    case WorkingMode::CLEAR_PATH:
      RCLCPP_INFO_STREAM(
        getLogger(),
        "[OdomTracker] setting working mode to CLEAR - clear_point_distance_threshold: "
          << clearPointDistanceThreshold_
          << ", clear_angular_distance_threshold: " << clearAngularDistanceThreshold_);
      break;
    case WorkingMode::IDLE:
      RCLCPP_INFO_STREAM(getLogger(), "[OdomTracker] setting working mode to IDLE");
      break;
    default:

      RCLCPP_INFO_STREAM(getLogger(), "[OdomTracker] setting working mode to <UNKNOWN>");
  }

  workingMode_ = workingMode;
  // RCLCPP_INFO(getLogger(),"odom_tracker m_mutex release");
}

/**
 ******************************************************************************************************************
 * setPublishMessages()
 ******************************************************************************************************************
 */
void OdomTracker::setPublishMessages(bool value)
{
  // RCLCPP_INFO(getLogger(),"odom_tracker m_mutex acquire");
  std::lock_guard<std::mutex> lock(m_mutex_);
  publishMessages = value;
  // RCLCPP_INFO(getLogger(),"odom_tracker m_mutex release");
  this->updateAggregatedStackPath();
}

void OdomTracker::pushPath() { this->pushPath(std::string()); }

void OdomTracker::pushPath(std::string pathname)
{
  RCLCPP_INFO(getLogger(), "odom_tracker m_mutex acquire");
  std::lock_guard<std::mutex> lock(m_mutex_);
  RCLCPP_INFO(getLogger(), "PUSH_PATH PATH EXITTING");
  this->logStateString();

  pathInfos_.push_back({currentPathName_, this->currentMotionGoal_});
  pathStack_.push_back(baseTrajectory_);

  currentPathName_ = pathname;

  // clean the current trajectory to start a new one
  baseTrajectory_.poses.clear();

  RCLCPP_INFO(getLogger(), "PUSH_PATH PATH EXITTING");
  this->logStateString();
  RCLCPP_INFO(getLogger(), "odom_tracker m_mutex release");
  this->updateAggregatedStackPath();

  this->currentMotionGoal_.reset();
}

void OdomTracker::popPath(int popCount, bool keepPreviousPath)
{
  RCLCPP_INFO(getLogger(), "odom_tracker m_mutex acquire");
  std::lock_guard<std::mutex> lock(m_mutex_);

  RCLCPP_INFO(getLogger(), "POP PATH ENTRY");
  this->logStateString();

  if (!keepPreviousPath)
  {
    baseTrajectory_.poses.clear();
  }

  while (popCount > 0 && !pathStack_.empty())
  {
    auto & stackedPath = pathStack_.back().poses;
    auto & stackedPathInfo = pathInfos_.back();

    baseTrajectory_.poses.insert(
      baseTrajectory_.poses.begin(), stackedPath.begin(), stackedPath.end());
    this->currentMotionGoal_ = stackedPathInfo.goalPose;
    pathStack_.pop_back();
    pathInfos_.pop_back();
    popCount--;

    RCLCPP_INFO(getLogger(), "POP PATH Iteration ");
    this->logStateString();
  }

  RCLCPP_INFO(getLogger(), "POP PATH EXITING");
  this->logStateString();
  RCLCPP_INFO(getLogger(), "odom_tracker m_mutex release");
  this->updateAggregatedStackPath();

  this->currentMotionGoal_.reset();
}

void OdomTracker::logStateString()
{
  std::stringstream ss;
  ss << "--- odom tracker state ---" << std::endl;
  ss
    << " - path stack -" << currentPathName_ << " -  size:"
    << pathStack_.size()
    //<< " goal: " << (getCurrentMotionGoal()? getCurrentMotionGoal()->pose: geometry_msgs::msg::Pose())
    << std::endl;
  ss << " - [STACK-HEAD active path size: " << baseTrajectory_.poses.size() << "]" << std::endl;
  int i = 0;
  for (auto & p : pathStack_ | boost::adaptors::reversed)
  {
    auto & pathinfo = pathInfos_[pathInfos_.size() - i - 1];
    std::string goalstr = "[]";
    if (pathinfo.goalPose)
    {
      std::stringstream ss;
      ss << *(pathinfo.goalPose);
      goalstr = ss.str();
    }

    ss << " - p " << i << "[" << p.header.stamp << "], size: " << p.poses.size() << " - "
       << pathinfo.name << " - goal: " << goalstr << std::endl;
    i++;
  }
  ss << "---";
  RCLCPP_INFO(getLogger(), ss.str().c_str());
}

void OdomTracker::clearPath()
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  baseTrajectory_.poses.clear();

  rtPublishPaths(getNode()->now());
  this->logStateString();
  this->updateAggregatedStackPath();

  this->currentMotionGoal_.reset();
}

void OdomTracker::setCurrentPathName(const std::string & currentPathName)
{
  currentPathName_ = currentPathName;
}

void OdomTracker::setStartPoint(const geometry_msgs::msg::PoseStamped & pose)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  RCLCPP_INFO_STREAM(getLogger(), "[OdomTracker] set current path starting point: " << pose);
  if (baseTrajectory_.poses.size() > 0)
  {
    baseTrajectory_.poses[0] = pose;
  }
  else
  {
    baseTrajectory_.poses.push_back(pose);
  }
  this->updateAggregatedStackPath();
}

void OdomTracker::setStartPoint(const geometry_msgs::msg::Pose & pose)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  RCLCPP_INFO_STREAM(getLogger(), "[OdomTracker] set current path starting point: " << pose);
  geometry_msgs::msg::PoseStamped posestamped;
  posestamped.header.frame_id = this->odomFrame_;
  posestamped.header.stamp = getNode()->now();
  posestamped.pose = pose;

  if (baseTrajectory_.poses.size() > 0)
  {
    baseTrajectory_.poses[0] = posestamped;
  }
  else
  {
    baseTrajectory_.poses.push_back(posestamped);
  }
  this->updateAggregatedStackPath();
}

void OdomTracker::setCurrentMotionGoal(const geometry_msgs::msg::PoseStamped & pose)
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  this->currentMotionGoal_ = pose;
}

std::optional<geometry_msgs::msg::PoseStamped> OdomTracker::getCurrentMotionGoal()
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  return this->currentMotionGoal_;
}

nav_msgs::msg::Path OdomTracker::getPath()
{
  std::lock_guard<std::mutex> lock(m_mutex_);
  return this->baseTrajectory_;
}

/**
 ******************************************************************************************************************
 * rtPublishPaths()
 ******************************************************************************************************************
 */
void OdomTracker::rtPublishPaths(rclcpp::Time timestamp)
{
  baseTrajectory_.header.stamp = timestamp;
  robotBasePathPub_->publish(baseTrajectory_);

  aggregatedStackPathMsg_.header.stamp = timestamp;
  robotBasePathStackedPub_->publish(aggregatedStackPathMsg_);
}

void OdomTracker::updateAggregatedStackPath()
{
  aggregatedStackPathMsg_.poses.clear();
  for (auto & p : pathStack_)
  {
    aggregatedStackPathMsg_.poses.insert(
      aggregatedStackPathMsg_.poses.end(), p.poses.begin(), p.poses.end());
  }

  aggregatedStackPathMsg_.header.frame_id = this->odomFrame_;
}

/**
 ******************************************************************************************************************
 * updateBackward()
 ******************************************************************************************************************
 */
bool OdomTracker::updateClearPath(const nav_msgs::msg::Odometry & odom)
{
  // we initially accept any message if the queue is empty
  /// Track robot base pose
  geometry_msgs::msg::PoseStamped base_pose;

  base_pose.pose = odom.pose.pose;
  base_pose.header = odom.header;
  baseTrajectory_.header = odom.header;

  bool acceptBackward = false;
  bool clearingError = false;
  bool finished = false;

  while (!finished)
  {
    if (
      baseTrajectory_.poses.size() <=
      1)  // we at least keep always the first point of the forward path when clearing
          // (this is important for backwards planner replanning and not losing the
          // last goal)
    {
      acceptBackward = false;
      finished = true;
    }
    else
    {
      auto & carrotPose = baseTrajectory_.poses.back().pose;
      auto & carrotPoint = carrotPose.position;

      double carrotAngle = tf2::getYaw(carrotPose.orientation);

      auto & currePose = base_pose.pose;
      auto & currePoint = currePose.position;
      double currentAngle = tf2::getYaw(currePose.orientation);

      double lastpointdist = p2pDistance(carrotPoint, currePoint);
      double goalAngleOffset = fabs(angles::shortest_angular_distance(carrotAngle, currentAngle));

      acceptBackward = !baseTrajectory_.poses.empty() &&
                       lastpointdist < clearPointDistanceThreshold_ &&
                       goalAngleOffset < clearAngularDistanceThreshold_;

      clearingError = lastpointdist > 2 * clearPointDistanceThreshold_;
      RCLCPP_DEBUG_STREAM(
        getLogger(), "[OdomTracker] clearing (accepted: " << acceptBackward
                                                          << ") linerr: " << lastpointdist
                                                          << ", anglerr: " << goalAngleOffset);
    }

    // RCLCPP_INFO(getLogger(),"Backwards, last distance: %lf < %lf accept: %d", dist,
    // minPointDistanceBackwardThresh_, acceptBackward);
    if (
      acceptBackward &&
      baseTrajectory_.poses.size() > 1) /* we always leave at least one item, specially interesting
                                                               for the backward local planner reach the backwards goal
                                                               with enough precision*/
    {
      baseTrajectory_.poses.pop_back();
    }
    else if (clearingError)
    {
      finished = true;
      RCLCPP_WARN(getLogger(), "[OdomTracker] Incorrect odom clearing motion.");
    }
    else
    {
      finished = true;
      /// Not removing point because it is enough far from the last cord point
    }
  }

  return acceptBackward;
}
/**
 ******************************************************************************************************************
 * updateRecordPath()
 ******************************************************************************************************************
 */
bool OdomTracker::updateRecordPath(const nav_msgs::msg::Odometry & odom)
{
  /// Track robot base pose
  geometry_msgs::msg::PoseStamped base_pose;

  base_pose.pose = odom.pose.pose;
  base_pose.header = odom.header;
  baseTrajectory_.header = odom.header;

  bool enqueueOdomMessage = false;

  double dist = -1;
  if (baseTrajectory_.poses.empty())
  {
    enqueueOdomMessage = true;
  }
  else
  {
    const auto & prevPose = baseTrajectory_.poses.back().pose;
    const geometry_msgs::msg::Point & prevPoint = prevPose.position;
    double prevAngle = tf2::getYaw(prevPose.orientation);

    const geometry_msgs::msg::Point & currePoint = base_pose.pose.position;
    double currentAngle = tf2::getYaw(base_pose.pose.orientation);

    dist = p2pDistance(prevPoint, currePoint);
    double goalAngleOffset = fabs(angles::shortest_angular_distance(prevAngle, currentAngle));

    // RCLCPP_WARN(getLogger(),"dist %lf vs min %lf", dist, recordPointDistanceThreshold_);

    if (dist > recordPointDistanceThreshold_ || goalAngleOffset > recordAngularDistanceThreshold_)
    {
      enqueueOdomMessage = true;
    }
    else
    {
      // RCLCPP_WARN(getLogger(),"skip odom, dist: %lf", dist);
      enqueueOdomMessage = false;
    }
  }

  if (enqueueOdomMessage)
  {
    baseTrajectory_.poses.push_back(base_pose);
  }

  return enqueueOdomMessage;
}

/**
 ******************************************************************************************************************
 * reconfigCB()
 ******************************************************************************************************************
 */
void OdomTracker::updateConfiguration()
{
  if (!getNode()->get_parameter("odom_frame", this->odomFrame_))
  {
  }

  if (!getNode()->get_parameter("record_point_distance_threshold", recordPointDistanceThreshold_))
  {
  }

  if (!getNode()->get_parameter(
        "record_angular_distance_threshold", recordAngularDistanceThreshold_))
  {
  }

  if (!getNode()->get_parameter("clear_point_distance_threshold", clearPointDistanceThreshold_))
  {
  }

  if (!getNode()->get_parameter("clear_angular_distance_threshold", clearAngularDistanceThreshold_))
  {
  }
}

/**
 ******************************************************************************************************************
 * processOdometryMessage()
 ******************************************************************************************************************
 */
void OdomTracker::processOdometryMessage(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  // RCLCPP_INFO(getLogger(),"odom_tracker m_mutex acquire");
  std::lock_guard<std::mutex> lock(m_mutex_);

  updateConfiguration();

  if (workingMode_ == WorkingMode::RECORD_PATH)
  {
    updateRecordPath(*odom);
  }
  else if (workingMode_ == WorkingMode::CLEAR_PATH)
  {
    updateClearPath(*odom);
  }

  // RCLCPP_WARN(getLogger(),"odomTracker odometry callback");
  if (publishMessages)
  {
    rtPublishPaths(odom->header.stamp);
  }

  // RCLCPP_INFO(getLogger(),"odom_tracker m_mutex release");
}
}  // namespace odom_tracker
}  // namespace cl_nav2z
