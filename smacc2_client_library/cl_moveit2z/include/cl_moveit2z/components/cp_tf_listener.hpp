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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <optional>

namespace cl_moveit2z
{
/**
 * @brief Component for shared TF2 transform management across all behaviors
 *
 * This component provides centralized access to TF2 transforms using static
 * resource sharing to avoid creating multiple tf2_ros::Buffer and
 * tf2_ros::TransformListener instances across different behaviors.
 *
 * Pattern: Static resource sharing (similar to nav2z_client's Pose component)
 */
class CpTfListener : public smacc2::ISmaccComponent
{
public:
  CpTfListener() = default;
  virtual ~CpTfListener() = default;

  /**
   * @brief Initialize the shared TF2 buffer and listener
   *
   * This method initializes the static tf2_ros::Buffer and tf2_ros::TransformListener
   * on first use. Subsequent component instances will reuse the same resources.
   */
  inline void onInitialize() override
  {
    std::lock_guard<std::mutex> guard(tfMutex_);

    if (!tfBuffer_)
    {
      RCLCPP_INFO(getLogger(), "[CpTfListener] Initializing shared TF2 buffer and listener");
      tfBuffer_ = std::make_shared<tf2_ros::Buffer>(getNode()->get_clock());
      tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    }
    else
    {
      RCLCPP_DEBUG(getLogger(), "[CpTfListener] Reusing existing shared TF2 resources");
    }
  }

  /**
   * @brief Thread-safe transform lookup
   *
   * @param target_frame Target frame for the transform
   * @param source_frame Source frame for the transform
   * @param time Time at which to get the transform (default: latest available)
   * @return std::optional<geometry_msgs::msg::TransformStamped> Transform if available, std::nullopt otherwise
   */
  inline std::optional<geometry_msgs::msg::TransformStamped> lookupTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time = rclcpp::Time(0))
  {
    std::lock_guard<std::mutex> guard(tfMutex_);

    if (!tfBuffer_)
    {
      RCLCPP_ERROR(
        getLogger(), "[CpTfListener] TF buffer not initialized. Call onInitialize() first.");
      return std::nullopt;
    }

    try
    {
      auto transform = tfBuffer_->lookupTransform(target_frame, source_frame, time);
      return transform;
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(
        getLogger(), "[CpTfListener] Could not get transform from '%s' to '%s': %s",
        source_frame.c_str(), target_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  /**
   * @brief Transform a pose to a different frame
   *
   * @param pose Input pose in its original frame
   * @param target_frame Target frame to transform the pose into
   * @return std::optional<geometry_msgs::msg::PoseStamped> Transformed pose if successful, std::nullopt otherwise
   */
  inline std::optional<geometry_msgs::msg::PoseStamped> transformPose(
    const geometry_msgs::msg::PoseStamped & pose, const std::string & target_frame)
  {
    std::lock_guard<std::mutex> guard(tfMutex_);

    if (!tfBuffer_)
    {
      RCLCPP_ERROR(
        getLogger(), "[CpTfListener] TF buffer not initialized. Call onInitialize() first.");
      return std::nullopt;
    }

    try
    {
      geometry_msgs::msg::PoseStamped transformed_pose;
      tfBuffer_->transform(pose, transformed_pose, target_frame);
      return transformed_pose;
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(
        getLogger(), "[CpTfListener] Could not transform pose to frame '%s': %s",
        target_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  /**
   * @brief Check if a transform is available
   *
   * @param target_frame Target frame
   * @param source_frame Source frame
   * @param time Time to check for (default: latest)
   * @param timeout How long to wait for the transform
   * @return bool True if transform is available, false otherwise
   */
  inline bool canTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time = rclcpp::Time(0),
    const rclcpp::Duration & timeout = rclcpp::Duration::from_seconds(0.0))
  {
    std::lock_guard<std::mutex> guard(tfMutex_);

    if (!tfBuffer_)
    {
      RCLCPP_ERROR(
        getLogger(), "[CpTfListener] TF buffer not initialized. Call onInitialize() first.");
      return false;
    }

    return tfBuffer_->canTransform(target_frame, source_frame, time, timeout);
  }

  /**
   * @brief Get raw access to the TF2 buffer for advanced use cases
   *
   * WARNING: Direct buffer access bypasses thread safety. Use with caution.
   *
   * @return std::shared_ptr<tf2_ros::Buffer> Shared pointer to the TF2 buffer
   */
  inline std::shared_ptr<tf2_ros::Buffer> getBuffer()
  {
    std::lock_guard<std::mutex> guard(tfMutex_);
    return tfBuffer_;
  }

private:
  // Shared static resources across all CpTfListener instances
  static std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  static std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  static std::mutex tfMutex_;
};

// Static member initialization
std::shared_ptr<tf2_ros::Buffer> CpTfListener::tfBuffer_ = nullptr;
std::shared_ptr<tf2_ros::TransformListener> CpTfListener::tfListener_ = nullptr;
std::mutex CpTfListener::tfMutex_;

}  // namespace cl_moveit2z
