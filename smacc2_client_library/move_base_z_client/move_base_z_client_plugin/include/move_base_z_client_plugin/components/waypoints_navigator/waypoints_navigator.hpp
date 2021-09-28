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

#include <move_base_z_client_plugin/components/waypoints_navigator/waypoints_event_dispatcher.hpp>
#include <move_base_z_client_plugin/move_base_z_client_plugin.hpp>
#include <smacc2/smacc.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace cl_move_base_z
{
class ClMoveBaseZ;

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

class WaypointNavigator : public smacc2::ISmaccComponent
{
public:
  WaypointEventDispatcher waypointsEventDispatcher;

  ClMoveBaseZ * client_;

  WaypointNavigator();

  void onInitialize() override;

  void insertWaypoint(int index, geometry_msgs::msg::Pose & newpose);

  void removeWaypoint(int index);

  void loadWayPointsFromFile(std::string filepath);

  void setWaypoints(const std::vector<geometry_msgs::msg::Pose> & waypoints);

  void setWaypoints(const std::vector<Pose2D> & waypoints);

  void sendNextGoal();

  const std::vector<geometry_msgs::msg::Pose> & getWaypoints() const;

  long getCurrentWaypointIndex() const;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    waypointsEventDispatcher.initialize<TSourceObject, TOrthogonal>(client_);
  }

  int currentWaypoint_;

private:
  void onGoalReached(ClMoveBaseZ::WrappedResult & res);

  std::vector<geometry_msgs::msg::Pose> waypoints_;

  boost::signals2::connection succeddedConnection_;
};
}  // namespace cl_move_base_z
