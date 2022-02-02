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

#include <tf2/transform_datatypes.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <nav2z_client/common.hpp>
#include <nav2z_client/components/goal_checker_switcher/goal_checker_switcher.hpp>
#include <nav2z_client/components/odom_tracker/odom_tracker.hpp>
#include <nav2z_client/components/planner_switcher/planner_switcher.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>
#include <nav2z_client/components/waypoints_navigator/waypoints_navigator.hpp>
#include <nav2z_client/nav2z_client.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cl_nav2z
{
using namespace std::chrono_literals;
WaypointNavigator::WaypointNavigator() : currentWaypoint_(0), waypoints_(0) {}

void WaypointNavigator::onInitialize() { client_ = dynamic_cast<ClNav2Z *>(owner_); }

void WaypointNavigator::onGoalCancelled(ClNav2Z::WrappedResult & /*res*/)
{
  stopWaitingResult();

  this->onNavigationRequestCancelled();
}

void WaypointNavigator::onGoalAborted(ClNav2Z::WrappedResult & /*res*/)
{
  stopWaitingResult();

  this->onNavigationRequestAborted();
}

void WaypointNavigator::onGoalReached(ClNav2Z::WrappedResult & /*res*/)
{
  waypointsEventDispatcher.postWaypointEvent(currentWaypoint_);
  currentWaypoint_++;
  RCLCPP_WARN(
    getLogger(), "[WaypointNavigator] Goal result received, incrementing waypoint index: %ld",
    currentWaypoint_);
  stopWaitingResult();

  onNavigationRequestSucceded();
}

void WaypointNavigator::rewind(int /*count*/)
{
  currentWaypoint_--;
  if (currentWaypoint_ < 0) currentWaypoint_ = 0;
}

void WaypointNavigator::stopWaitingResult()
{
  if (succeddedNav2ZClientConnection_.connected())
  {
    this->succeddedNav2ZClientConnection_.disconnect();
    this->cancelledNav2ZClientConnection_.disconnect();
    this->abortedNav2ZClientConnection_.disconnect();
  }
}

void WaypointNavigator::sendNextGoal()
{
  if (currentWaypoint_ >= 0 && currentWaypoint_ < (int)waypoints_.size())
  {
    auto & next = waypoints_[currentWaypoint_];

    ClNav2Z::Goal goal;
    auto p = client_->getComponent<cl_nav2z::Pose>();
    auto pose = p->toPoseMsg();

    // configuring goal
    goal.pose.header.frame_id = p->getReferenceFrame();
    goal.pose.header.stamp = getNode()->now();
    goal.pose.pose = next;

    RCLCPP_WARN(getLogger(), "[WaypointsNavigator] Configuring default planners");
    auto plannerSwitcher = client_->getComponent<PlannerSwitcher>();
    plannerSwitcher->setDefaultPlanners();

    RCLCPP_WARN(getLogger(), "[WaypointsNavigator] Configuring default goal planner");
    auto goalCheckerSwitcher = client_->getComponent<GoalCheckerSwitcher>();
    goalCheckerSwitcher->setGoalCheckerId("goal_checker");

    // publish stuff
    // rclcpp::sleep_for(5s);

    RCLCPP_INFO(getLogger(), "[WaypointsNavigator] Getting odom tracker");
    auto odomTracker = client_->getComponent<cl_nav2z::odom_tracker::OdomTracker>();
    if (odomTracker != nullptr)
    {
      RCLCPP_INFO(getLogger(), "[WaypointsNavigator] Storing path in odom tracker");

      auto pathname =
        this->owner_->getStateMachine()->getCurrentState()->getName() + " - " + getName();
      odomTracker->pushPath(pathname);
      odomTracker->setStartPoint(pose);
      odomTracker->setWorkingMode(cl_nav2z::odom_tracker::WorkingMode::RECORD_PATH);
    }

    // SEND GOAL
    if (!succeddedNav2ZClientConnection_.connected())
    {
      this->succeddedNav2ZClientConnection_ =
        client_->onSucceeded(&WaypointNavigator::onGoalReached, this);
      this->cancelledNav2ZClientConnection_ =
        client_->onAborted(&WaypointNavigator::onGoalCancelled, this);
      this->abortedNav2ZClientConnection_ =
        client_->onCancelled(&WaypointNavigator::onGoalAborted, this);
    }

    client_->sendGoal(goal);
  }
  else
  {
    RCLCPP_WARN(
      getLogger(),
      "[WaypointsNavigator] All waypoints were consumed. There is no more waypoints available.");
  }
}

void WaypointNavigator::insertWaypoint(int index, geometry_msgs::msg::Pose & newpose)
{
  if (index >= 0 && index <= (int)waypoints_.size())
  {
    waypoints_.insert(waypoints_.begin(), index, newpose);
  }
}

void WaypointNavigator::setWaypoints(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  this->waypoints_ = waypoints;
}

void WaypointNavigator::setWaypoints(const std::vector<Pose2D> & waypoints)
{
  waypoints_.clear();
  waypointsNames_.clear();
  int i = 0;
  for (auto & p : waypoints)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = p.x_;
    pose.position.y = p.y_;
    pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, p.yaw_);
    pose.orientation = tf2::toMsg(q);

    waypoints_.push_back(pose);
    waypointsNames_.push_back(std::to_string(i++));
  }
}

void WaypointNavigator::removeWaypoint(int index)
{
  if (index >= 0 && index < (int)waypoints_.size())
  {
    waypoints_.erase(waypoints_.begin() + index);
  }
}

const std::vector<geometry_msgs::msg::Pose> & WaypointNavigator::getWaypoints() const
{
  return waypoints_;
}

long WaypointNavigator::getCurrentWaypointIndex() const { return currentWaypoint_; }

#define HAVE_NEW_YAMLCPP
void WaypointNavigator::loadWayPointsFromFile(std::string filepath)
{
  RCLCPP_INFO_STREAM(getLogger(), "[WaypointNavigator] Loading file:" << filepath);
  this->waypoints_.clear();
  std::ifstream ifs(filepath.c_str(), std::ifstream::in);
  if (ifs.good() == false)
  {
    throw std::string("Waypoints file not found");
  }

  try
  {
#ifdef HAVE_NEW_YAMLCPP
    YAML::Node node = YAML::Load(ifs);
#else
    YAML::Parser parser(ifs);
    parser.GetNextDocument(node);
#endif

#ifdef HAVE_NEW_YAMLCPP
    const YAML::Node & wp_node_tmp = node["waypoints"];
    const YAML::Node * wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
#else
    const YAML::Node * wp_node = node.FindValue("waypoints");
#endif

    if (wp_node != NULL)
    {
      for (unsigned int i = 0; i < wp_node->size(); ++i)
      {
        // Parse waypoint entries on YAML
        geometry_msgs::msg::Pose wp;

        try
        {
          // (*wp_node)[i]["name"] >> wp.name;
          // (*wp_node)[i]["frame_id"] >> wp.header.frame_id;
          wp.position.x = (*wp_node)[i]["position"]["x"].as<double>();
          wp.position.y = (*wp_node)[i]["position"]["y"].as<double>();
          wp.position.z = (*wp_node)[i]["position"]["z"].as<double>();
          wp.orientation.x = (*wp_node)[i]["orientation"]["x"].as<double>();
          wp.orientation.y = (*wp_node)[i]["orientation"]["y"].as<double>();
          wp.orientation.z = (*wp_node)[i]["orientation"]["z"].as<double>();
          wp.orientation.w = (*wp_node)[i]["orientation"]["w"].as<double>();

          this->waypoints_.push_back(wp);
        }
        catch (...)
        {
          RCLCPP_ERROR(getLogger(), "parsing waypoint file, syntax error in point %d", i);
        }
      }
      RCLCPP_INFO_STREAM(getLogger(), "Parsed " << this->waypoints_.size() << " waypoints.");
    }
    else
    {
      RCLCPP_WARN_STREAM(getLogger(), "Couldn't find any waypoints in the provided yaml file.");
    }
  }
  catch (const YAML::ParserException & ex)
  {
    RCLCPP_ERROR_STREAM(
      getLogger(), "Error loading the Waypoints YAML file. Incorrect syntax: " << ex.what());
  }
}

void WaypointNavigator::loadWayPointsFromFile2(std::string filepath)
{
  RCLCPP_INFO_STREAM(getLogger(), "[WaypointNavigator] Loading file:" << filepath);
  this->waypoints_.clear();
  std::ifstream ifs(filepath.c_str(), std::ifstream::in);
  if (ifs.good() == false)
  {
    throw std::string("Waypoints file not found");
  }

  try
  {
#ifdef HAVE_NEW_YAMLCPP
    YAML::Node node = YAML::Load(ifs);
#else
    YAML::Parser parser(ifs);
    parser.GetNextDocument(node);
#endif

#ifdef HAVE_NEW_YAMLCPP
    const YAML::Node & wp_node_tmp = node["waypoints"];
    const YAML::Node * wp_node = wp_node_tmp ? &wp_node_tmp : NULL;
#else
    const YAML::Node * wp_node = node.FindValue("waypoints");
#endif

    if (wp_node != NULL)
    {
      for (unsigned int i = 0; i < wp_node->size(); ++i)
      {
        // Parse waypoint entries on YAML
        geometry_msgs::msg::Pose wp;

        try
        {
          // (*wp_node)[i]["name"] >> wp.name;
          // (*wp_node)[i]["frame_id"] >> wp.header.frame_id;
          wp.position.x = (*wp_node)[i]["x"].as<double>();
          wp.position.y = (*wp_node)[i]["y"].as<double>();
          auto name = (*wp_node)[i]["name"].as<std::string>();

          this->waypoints_.push_back(wp);
          this->waypointsNames_.push_back(name);
        }
        catch (...)
        {
          RCLCPP_ERROR(getLogger(), "parsing waypoint file, syntax error in point %d", i);
        }
      }
      RCLCPP_INFO_STREAM(getLogger(), "Parsed " << this->waypoints_.size() << " waypoints.");
    }
    else
    {
      RCLCPP_WARN_STREAM(getLogger(), "Couldn't find any waypoints in the provided yaml file.");
    }
  }
  catch (const YAML::ParserException & ex)
  {
    RCLCPP_ERROR_STREAM(
      getLogger(), "Error loading the Waypoints YAML file. Incorrect syntax: " << ex.what());
  }
}
}  // namespace cl_nav2z
