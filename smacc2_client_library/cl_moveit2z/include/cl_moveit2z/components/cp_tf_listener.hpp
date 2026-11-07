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
 * This component provides centralized access to TF2 transforms: all instances in a process
 * share one tf2_ros::Buffer and one tf2_ros::TransformListener, owned by the instances
 * themselves (process-wide weak references), so the pair lives exactly as long as some
 * CpTfListener does and is destroyed before the rclcpp context.
 *
 * Pattern: Static resource sharing (similar to nav2z_client's Pose component)
 */
class CpTfListener : public smacc2::ISmaccComponent
{
public:
  CpTfListener() = default;
  virtual ~CpTfListener() = default;

  /**
   * @brief Take, or create, the shared TF2 buffer and listener
   *
   * The first instance creates the tf2_ros::Buffer and tf2_ros::TransformListener; later
   * instances created while one is alive share them. Each instance holds its own strong
   * reference.
   */
  inline void onInitialize() override
  {
    // Take strong references to the process-wide buffer and listener, creating them if no
    // other instance holds them. This instance then owns them for its own lifetime; the statics
    // below are weak, so the pair is destroyed with the last CpTfListener, inside the state
    // machine's destruction and before the rclcpp context is finalized. A static shared_ptr used
    // to hold the listener until static destruction after main() returned, when its node's
    // teardown ran against a finalized DDS participant and crashed (finding R, cl_nav2z sibling).
    std::lock_guard<std::mutex> guard(tfMutex_);
    tfBuffer_ = sharedTfBuffer_.lock();
    tfListener_ = sharedTfListener_.lock();
    if (!tfBuffer_ || !tfListener_)
    {
      RCLCPP_INFO(getLogger(), "[CpTfListener] Initializing shared TF2 buffer and listener");
      tfBuffer_ = std::make_shared<tf2_ros::Buffer>(getNode()->get_clock());
      tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
      sharedTfBuffer_ = tfBuffer_;
      sharedTfListener_ = tfListener_;
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
  // Per-instance strong references. Buffer before listener: the listener references the buffer
  // and must be destroyed first.
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  // Process-wide weak references so every instance shares one buffer and listener while any
  // instance is alive, without extending their lifetime past the last instance. `inline static`
  // so a header-only component defines each once per program, not once per translation unit.
  inline static std::weak_ptr<tf2_ros::Buffer> sharedTfBuffer_;
  inline static std::weak_ptr<tf2_ros::TransformListener> sharedTfListener_;
  inline static std::mutex tfMutex_;
};

}  // namespace cl_moveit2z
