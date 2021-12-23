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

#include <nav2z_client/components/waypoints_navigator/waypoints_event_dispatcher.hpp>
#include <nav2z_client/nav2z_client.hpp>
#include <smacc2/smacc.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace cl_nav2z
{
class ClNav2Z;

struct Pose2D
{
  Pose2D(double x, double y, double yaw)
  {
    this->x_ = x;
    this->y_ = y;
    this->yaw_ = yaw;
  }

  double x_;
  double y_;
  double yaw_;
};

// This component contains a list of waypoints. These waypoints can
// be iterated in the different states using CbNextWaiPoint
// waypoint index is only incremented if the current waypoint is successfully reached
class WaypointNavigator : public smacc2::ISmaccComponent
{
public:
  WaypointEventDispatcher waypointsEventDispatcher;

  ClNav2Z * client_;

  WaypointNavigator();

  void onInitialize() override;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    waypointsEventDispatcher.initialize<TSourceObject, TOrthogonal>(client_);
  }

  void loadWayPointsFromFile(std::string filepath);

  void loadWayPointsFromFile2(std::string filepath);

  void setWaypoints(const std::vector<geometry_msgs::msg::Pose> & waypoints);

  void setWaypoints(const std::vector<Pose2D> & waypoints);

  void sendNextGoal();
  void stopWaitingResult();

  const std::vector<geometry_msgs::msg::Pose> & getWaypoints() const;

  long getCurrentWaypointIndex() const;

  long currentWaypoint_;

  void rewind(int count);

  smacc2::SmaccSignal<void()> onNavigationRequestSucceded;
  smacc2::SmaccSignal<void()> onNavigationRequestAborted;
  smacc2::SmaccSignal<void()> onNavigationRequestCancelled;

private:
  void insertWaypoint(int index, geometry_msgs::msg::Pose & newpose);

  void removeWaypoint(int index);

  void onGoalReached(ClNav2Z::WrappedResult & res);
  void onGoalCancelled(ClNav2Z::WrappedResult & /*res*/);
  void onGoalAborted(ClNav2Z::WrappedResult & /*res*/);

  std::vector<geometry_msgs::msg::Pose> waypoints_;

  std::vector<std::string> waypointsNames_;

  boost::signals2::connection succeddedNav2ZClientConnection_;
  boost::signals2::connection abortedNav2ZClientConnection_;
  boost::signals2::connection cancelledNav2ZClientConnection_;
};
}  // namespace cl_nav2z
