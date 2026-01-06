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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <smacc2/client_core_components/cp_topic_subscriber.hpp>
#include <smacc2/component.hpp>
#include <smacc2/smacc_signal.hpp>

#include <map>
#include <mutex>
#include <string>

namespace cl_isaac_apriltag
{

using AprilTagDetectionArray = isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray;

/**
 * @brief Component that tracks AprilTag detections and transforms them to a target frame.
 *
 * This component uses requiresComponent() to get the CpTopicSubscriber and hooks into
 * its onMessageReceived signal. It handles TF lookups to transform tag detections to
 * the target frame (default: "map") and provides time-filtered queries.
 */
class CpAprilTagTracker : public smacc2::ISmaccComponent
{
public:
  CpAprilTagTracker(std::string target_frame = "map") : targetFrame_(target_frame) {}

  virtual ~CpAprilTagTracker() {}

  void onInitialize() override
  {
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->getNode()->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
  }

  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    // Get the topic subscriber component via requiresComponent pattern
    this->requiresComponent(topicSubscriber_);

    // Hook into the subscriber's message received signal
    topicSubscriber_->onMessageReceived(&CpAprilTagTracker::onAprilTagMessage, this);
  }

  /**
   * @brief Get tags detected within the specified duration from now.
   * @param duration Maximum age of detections to include
   * @return Map of tag frame IDs to their poses in the target frame
   */
  std::map<std::string, geometry_msgs::msg::PoseStamped> getTagsWithinTime(
    rclcpp::Duration duration)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::map<std::string, geometry_msgs::msg::PoseStamped> ret;

    auto now = getNode()->now();

    std::stringstream ss;
    for (auto & apriltag : detectedAprilTagsMapPose_)
    {
      auto tagstamp = rclcpp::Time(apriltag.second.header.stamp);
      rclcpp::Duration elapsed = now - tagstamp;

      ss << "[CpAprilTagTracker] AprilTag: " << apriltag.first << " Elapsed: " << elapsed.seconds()
         << " Stamp: " << tagstamp.seconds() << " now " << now.seconds() << std::endl;
      if (elapsed < duration)
      {
        ret.insert(apriltag);
        ss << "[SELECTED]" << std::endl;
      }
      else
      {
        ss << "[NOT SELECTED]" << std::endl;
      }
      ss << std::endl;
    }

    RCLCPP_INFO_THROTTLE(getLogger(), *(getNode()->get_clock()), 1000, "%s", ss.str().c_str());
    return ret;
  }

  /**
   * @brief Get a specific tag's pose by frame ID.
   * @param tag_frame_id The tag frame ID (e.g., "36h11:5")
   * @return Optional pose if tag is tracked, nullopt otherwise
   */
  std::optional<geometry_msgs::msg::PoseStamped> getTagPose(const std::string & tag_frame_id)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = detectedAprilTagsMapPose_.find(tag_frame_id);
    if (it != detectedAprilTagsMapPose_.end())
    {
      return it->second;
    }
    return std::nullopt;
  }

  // Signal emitted when AprilTag detections are processed
  smacc2::SmaccSignal<void(const AprilTagDetectionArray::SharedPtr &)> onAprilTagDetection_;

  /**
   * @brief Subscribe to AprilTag detection events.
   * @param callback Member function to call when detections occur
   * @param object Object instance for the callback
   * @return Connection object for managing the subscription
   */
  template <typename T>
  smacc2::SmaccSignalConnection onAprilTagDetected(
    void (T::*callback)(const AprilTagDetectionArray::SharedPtr &), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onAprilTagDetection_, callback, object);
  }

private:
  // Topic subscriber component (obtained via requiresComponent)
  smacc2::client_core_components::CpTopicSubscriber<AprilTagDetectionArray> * topicSubscriber_ =
    nullptr;

  // TF management
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::string targetFrame_;

  // Detection storage (thread-safe)
  std::map<std::string, geometry_msgs::msg::PoseStamped> detectedAprilTagsMapPose_;
  std::mutex mutex_;

  void onAprilTagMessage(const AprilTagDetectionArray & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    std::stringstream ss;
    for (auto & detection : msg.detections)
    {
      std::string apriltag_frameid = detection.family + ":" + std::to_string(detection.id);
      ss << "[CpAprilTagTracker] AprilTag detected: " << apriltag_frameid << std::endl;

      // Get map position using TF lookup
      geometry_msgs::msg::TransformStamped transformStampedGlobal;

      try
      {
        transformStampedGlobal =
          tfBuffer_->lookupTransform(targetFrame_, apriltag_frameid, msg.header.stamp);
      }
      catch (tf2::TransformException & ex)
      {
        RCLCPP_WARN_THROTTLE(
          getLogger(), *(getNode()->get_clock()), 1000,
          "[CpAprilTagTracker] TF lookup failed for %s: %s", apriltag_frameid.c_str(), ex.what());
        continue;
      }

      // Transform to pose
      geometry_msgs::msg::PoseStamped poseStamped;
      poseStamped.header = transformStampedGlobal.header;
      poseStamped.pose.position.x = transformStampedGlobal.transform.translation.x;
      poseStamped.pose.position.y = transformStampedGlobal.transform.translation.y;
      poseStamped.pose.position.z = transformStampedGlobal.transform.translation.z;
      poseStamped.pose.orientation = transformStampedGlobal.transform.rotation;

      detectedAprilTagsMapPose_[apriltag_frameid] = poseStamped;

      ss << "[CpAprilTagTracker] new AprilTag detected: " << apriltag_frameid << " at "
         << poseStamped.pose.position.x << ", " << poseStamped.pose.position.y << ", "
         << poseStamped.pose.position.z;
    }
    RCLCPP_INFO_STREAM_THROTTLE(getLogger(), *(getNode()->get_clock()), 1000, ss.str());

    // Emit signal with SharedPtr for downstream subscribers
    auto msgPtr = std::make_shared<AprilTagDetectionArray>(msg);
    onAprilTagDetection_(msgPtr);
  }
};

}  // namespace cl_isaac_apriltag
