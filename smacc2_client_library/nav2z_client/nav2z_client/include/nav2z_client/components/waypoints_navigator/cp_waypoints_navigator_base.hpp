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

#include <nav2z_client/components/waypoints_navigator/cp_waypoints_event_dispatcher.hpp>
#include <nav2z_client/nav2z_client.hpp>
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

  virtual ~CpWaypointNavigatorBase();

  void onInitialize() override;

  // DEPRECATED: For third-party compatibility only. Third-party developers should migrate to onStateOrthogonalAllocation
  // This method exists to support existing third-party classes that inherit from this base class
  // and call CpWaypointNavigatorBase::onOrthogonalAllocation<TOrthogonal, TSourceObject>()
  template <typename TOrthogonal, typename TSourceObject>
  [[deprecated(
    "Use onStateOrthogonalAllocation instead. This method exists only for third-party "
    "compatibility.")]] void
  onOrthogonalAllocation()
  {
    // Call the new method to maintain functionality for third-party inheritors
    onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    ClNav2Z * client = dynamic_cast<ClNav2Z *>(owner_);
    waypointsEventDispatcher.initialize<TSourceObject, TOrthogonal>(client);
  }

  void loadWayPointsFromFile(std::string filepath);

  void loadWayPointsFromFile2(std::string filepath);

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

  void rewind(int count);

  void forward(int count);
  void seekName(std::string name);

  void loadWaypointsFromYamlParameter(
    std::string parameter_name, std::string yaml_file_package_name);

  void notifyGoalReached();

protected:
  void insertWaypoint(int index, geometry_msgs::msg::Pose & newpose);

  void removeWaypoint(int index);

  std::vector<geometry_msgs::msg::Pose> waypoints_;

  std::vector<std::string> waypointsNames_;
};
}  // namespace cl_nav2z
