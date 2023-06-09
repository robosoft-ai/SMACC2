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

#include <nav2z_client/components/waypoints_navigator/cp_waypoints_navigator.hpp>
#include <nav2z_client/nav2z_client.hpp>

#include <smacc2/smacc.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace cl_nav2z
{
class ClNav2Z;

class CpWaypointsVisualizer : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  cl_nav2z::CpWaypointNavigator * waypointsNavigator_;

  CpWaypointsVisualizer(rclcpp::Duration duration);

  void onInitialize() override;

protected:
  virtual void update() override;

private:
  std::mutex m_mutex_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markersPub_;
  visualization_msgs::msg::MarkerArray markers_;
  visualization_msgs::msg::MarkerArray markerLabels_;

  void createMarker(const geometry_msgs::msg::Pose & waypoint, visualization_msgs::msg::Marker & m);
  void createMarkerLabel(
    const geometry_msgs::msg::Pose & waypoint, std::string label,
    visualization_msgs::msg::Marker & m);
};
}  // namespace cl_nav2z
