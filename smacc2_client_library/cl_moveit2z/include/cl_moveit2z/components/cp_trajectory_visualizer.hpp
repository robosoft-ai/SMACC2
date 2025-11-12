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
 *****************************************************************************************************************/

#pragma once

#include <smacc2/component.hpp>
#include <smacc2/smacc_updatable.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace cl_moveit2z
{
/**
 * @brief Component for visualizing trajectories as RViz markers
 *
 * This component publishes trajectory paths as visualization markers that can be
 * viewed in RViz. It follows the ISmaccUpdatable pattern for periodic publishing.
 *
 * Pattern: Publisher + ISmaccUpdatable (similar to nav2z_client's CpWaypointsVisualizer)
 */
class CpTrajectoryVisualizer : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  /**
   * @brief Constructor
   *
   * @param publishRate Publishing rate (default: 10Hz)
   */
  CpTrajectoryVisualizer(double publishRate = 10.0) : publishRate_(publishRate), enabled_(false) {}

  virtual ~CpTrajectoryVisualizer() = default;

  /**
   * @brief Initialize the marker publisher
   */
  inline void onInitialize() override
  {
    markersPub_ =
      getNode()->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 1);
    RCLCPP_INFO(getLogger(), "[CpTrajectoryVisualizer] Initialized");

    // Initialize timing for publish rate control
    lastPublishTime_ = getNode()->now();
  }

  /**
   * @brief Periodic update - publishes markers if enabled
   */
  inline void update() override
  {
    std::lock_guard<std::mutex> guard(markersMutex_);

    if (!enabled_ || !markersReady_)
    {
      return;
    }

    // Rate limiting
    auto now = getNode()->now();
    auto elapsed = (now - lastPublishTime_).seconds();
    if (elapsed < (1.0 / publishRate_))
    {
      return;
    }

    markersPub_->publish(markers_);
    lastPublishTime_ = now;
  }

  /**
   * @brief Set trajectory to visualize
   *
   * @param poses Vector of poses defining the trajectory path
   * @param ns Namespace for the markers (default: "trajectory")
   */
  inline void setTrajectory(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses,
    const std::string & ns = "trajectory")
  {
    std::lock_guard<std::mutex> guard(markersMutex_);

    RCLCPP_INFO(
      getLogger(), "[CpTrajectoryVisualizer] Setting trajectory with %lu poses", poses.size());

    markers_.markers.clear();

    if (poses.empty())
    {
      RCLCPP_WARN(getLogger(), "[CpTrajectoryVisualizer] Empty trajectory provided");
      markersReady_ = false;
      return;
    }

    // Create spheres for each pose
    for (size_t i = 0; i < poses.size(); ++i)
    {
      visualization_msgs::msg::Marker marker;
      marker.header = poses[i].header;
      marker.ns = ns;
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose = poses[i].pose;

      marker.scale.x = markerScale_;
      marker.scale.y = markerScale_;
      marker.scale.z = markerScale_;

      marker.color.r = markerColor_[0];
      marker.color.g = markerColor_[1];
      marker.color.b = markerColor_[2];
      marker.color.a = markerColor_[3];

      markers_.markers.push_back(marker);
    }

    // Create line strip connecting all poses
    if (poses.size() > 1)
    {
      visualization_msgs::msg::Marker lineMarker;
      lineMarker.header = poses[0].header;
      lineMarker.ns = ns + "_path";
      lineMarker.id = static_cast<int>(poses.size());
      lineMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      lineMarker.action = visualization_msgs::msg::Marker::ADD;

      lineMarker.scale.x = markerScale_ * 0.3;  // Line width

      lineMarker.color.r = markerColor_[0];
      lineMarker.color.g = markerColor_[1];
      lineMarker.color.b = markerColor_[2];
      lineMarker.color.a = markerColor_[3] * 0.7;  // Slightly transparent

      for (const auto & pose : poses)
      {
        lineMarker.points.push_back(pose.pose.position);
      }

      markers_.markers.push_back(lineMarker);
    }

    markersReady_ = true;
    enabled_ = true;

    RCLCPP_INFO(
      getLogger(), "[CpTrajectoryVisualizer] Created %lu markers", markers_.markers.size());
  }

  /**
   * @brief Clear all markers
   */
  inline void clearMarkers()
  {
    std::lock_guard<std::mutex> guard(markersMutex_);

    RCLCPP_INFO(getLogger(), "[CpTrajectoryVisualizer] Clearing markers");

    // Send delete action for all markers
    for (auto & marker : markers_.markers)
    {
      marker.action = visualization_msgs::msg::Marker::DELETE;
    }

    if (!markers_.markers.empty())
    {
      markersPub_->publish(markers_);
    }

    markers_.markers.clear();
    markersReady_ = false;
    enabled_ = false;
  }

  /**
   * @brief Set marker color (RGBA values 0.0-1.0)
   */
  inline void setColor(float r, float g, float b, float a = 1.0)
  {
    markerColor_[0] = r;
    markerColor_[1] = g;
    markerColor_[2] = b;
    markerColor_[3] = a;
  }

  /**
   * @brief Set marker scale (sphere diameter in meters)
   */
  inline void setScale(double scale) { markerScale_ = scale; }

  /**
   * @brief Enable/disable marker publishing
   */
  inline void setEnabled(bool enabled)
  {
    std::lock_guard<std::mutex> guard(markersMutex_);
    enabled_ = enabled;
  }

  /**
   * @brief Check if visualizer is enabled
   */
  inline bool isEnabled() const { return enabled_; }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markersPub_;
  visualization_msgs::msg::MarkerArray markers_;
  std::mutex markersMutex_;

  bool markersReady_ = false;
  bool enabled_ = false;

  // Marker appearance
  double markerScale_ = 0.02;                        // 2cm spheres
  float markerColor_[4] = {0.0f, 1.0f, 0.0f, 0.8f};  // Green, slightly transparent

  // Publishing rate control
  double publishRate_;
  rclcpp::Time lastPublishTime_;
};

}  // namespace cl_moveit2z
