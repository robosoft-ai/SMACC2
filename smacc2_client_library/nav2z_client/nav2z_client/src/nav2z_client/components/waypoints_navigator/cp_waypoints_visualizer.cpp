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

#include <nav2z_client/components/waypoints_navigator/cp_waypoints_visualizer.hpp>

namespace cl_nav2z
{
const std::string frameid = "map";

CpWaypointsVisualizer::CpWaypointsVisualizer(rclcpp::Duration duration) : ISmaccUpdatable(duration)
{
}

void CpWaypointsVisualizer::onInitialize()
{
  markersPub_ = getNode()->create_publisher<visualization_msgs::msg::MarkerArray>(
    "cp_waypoints_visualizer/visualization_markers", rclcpp::QoS(rclcpp::KeepLast(1)));

  this->requiresComponent(waypointsNavigator_);
  auto & waypoints = waypointsNavigator_->getWaypoints();
  auto & waypointsNames = waypointsNavigator_->getWaypointNames();

  int i = 0;
  for (auto & waypoint : waypoints)
  {
    std::string name;
    if ((long)waypointsNames.size() > i)
    {
      name = waypointsNames[i];
    }
    else
    {
      name = "waypoint_" + std::to_string(i);
    }

    visualization_msgs::msg::Marker marker;
    createMarker(waypoint, marker);
    markers_.markers.push_back(marker);

    visualization_msgs::msg::Marker markerlabel;
    createMarkerLabel(waypoint, name, markerlabel);
    markerLabels_.markers.push_back(markerlabel);

    i++;
  }
}

void CpWaypointsVisualizer::createMarkerLabel(
  const geometry_msgs::msg::Pose & waypoint, std::string label,
  visualization_msgs::msg::Marker & marker)
{
  marker.header.frame_id = frameid;
  marker.header.stamp = getNode()->now();
  marker.ns = "waypoints_labels";

  marker.id = markers_.markers.size();
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.text = label;

  marker.color.a = 1.0;
  marker.pose = waypoint;
  marker.pose.position.z += 0.3;
}

void CpWaypointsVisualizer::createMarker(
  const geometry_msgs::msg::Pose & waypoint, visualization_msgs::msg::Marker & marker)
{
  marker.header.frame_id = frameid;
  marker.header.stamp = getNode()->now();
  marker.ns = "waypoints";

  marker.id = markers_.markers.size();
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.a = 1.0;
  marker.pose = waypoint;
}

void CpWaypointsVisualizer::update()
{
  std::lock_guard<std::mutex> guard(m_mutex_);

  auto index = waypointsNavigator_->getCurrentWaypointIndex();

  int i = 0;

  for (auto & marker : markers_.markers)
  {
    marker.header.stamp = getNode()->now();

    if (i >= index)
    {
      marker.color.r = 1.0;
      marker.color.g = 0;
      marker.color.b = 0;
    }
    else
    {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0;
    }
    i++;
  }

  i = 0;
  for (auto & marker : markerLabels_.markers)
  {
    marker.header.stamp = getNode()->now();

    if (i >= index)
    {
      marker.color.r = 1.0;
      marker.color.g = 0;
      marker.color.b = 0;
    }
    else
    {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0;
    }
    i++;
  }

  //markers_.header.stamp = getNode()->now();
  markersPub_->publish(markers_);
  markersPub_->publish(markerLabels_);
}

}  // namespace cl_nav2z
