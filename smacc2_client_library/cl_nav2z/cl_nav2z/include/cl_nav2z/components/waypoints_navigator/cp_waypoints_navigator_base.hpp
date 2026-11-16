// Copyright 2025 Robosoft Inc.
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

#include <cl_nav2z/cl_nav2z.hpp>
#include <cl_nav2z/components/waypoints_navigator/cp_waypoints_event_dispatcher.hpp>
#include <smacc2/smacc.hpp>

#include <geometry_msgs/msg/pose.hpp>

namespace cl_nav2z
{
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
class CpWaypointNavigatorBase : public smacc2::ISmaccComponent
{
public:
  WaypointEventDispatcher waypointsEventDispatcher;

  CpWaypointNavigatorBase();

  ~CpWaypointNavigatorBase() override;

  void onInitialize() override;

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    ClNav2Z * client = dynamic_cast<ClNav2Z *>(owner_);
    waypointsEventDispatcher.initialize<TSourceObject, TOrthogonal>(client);
  }

  // Both return false, with the list empty, when the file cannot be opened or is not valid YAML.
  bool loadWayPointsFromFile(std::string filepath);

  bool loadWayPointsFromFile2(std::string filepath);

  void setWaypoints(const std::vector<geometry_msgs::msg::Pose> & waypoints);

  void setWaypoints(const std::vector<Pose2D> & waypoints);

  const std::vector<geometry_msgs::msg::Pose> & getWaypoints() const;
  const std::vector<std::string> & getWaypointNames() const;
  std::optional<geometry_msgs::msg::Pose> getNamedPose(std::string name) const;
  geometry_msgs::msg::Pose getPose(int index) const;
  geometry_msgs::msg::Pose getCurrentPose() const;

  long getCurrentWaypointIndex() const;
  std::optional<std::string> getCurrentWaypointName() const;

  long currentWaypoint_;

  // Move the index `count` waypoints back or forward, stopping at the first and the last.
  void rewind(int count);

  void forward(int count);
  // Moves the index to the waypoint of that name; false, with the index unchanged, if there is none.
  bool seekName(std::string name);

  bool loadWaypointsFromYamlParameter(
    std::string parameter_name, std::string yaml_file_package_name);

  void notifyGoalReached();

protected:
  void insertWaypoint(int index, geometry_msgs::msg::Pose & newpose);

  void removeWaypoint(int index);

  // The two lists always have the same length; a waypoint with no name has an empty one.
  std::vector<geometry_msgs::msg::Pose> waypoints_;

  std::vector<std::string> waypointsNames_;

private:
  void addWaypoint(const geometry_msgs::msg::Pose & pose, const std::string & name);

  void clearWaypoints();
};
}  // namespace cl_nav2z
