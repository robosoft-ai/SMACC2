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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <isaac_ros_apriltag_interfaces/msg/april_tag_detection_array.hpp>
#include <smacc2/smacc.hpp>
#include <smacc2/smacc_client.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cl_isaac_apriltag
{
template <typename AsyncCB, typename Orthogonal>
struct EvUnvisitedAprilTagDetected : sc::event<EvUnvisitedAprilTagDetected<AsyncCB, Orthogonal>>
{
};

class ClIsaacApriltag : public smacc2::ISmaccClient
{
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  rclcpp::Subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>::SharedPtr
    apriltagSub_;

  smacc2::SmaccSignal<void(
    const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr)>
    onAprilTagDetection_;

public:
  ClIsaacApriltag() {}

  void onInitialize() override
  {
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->getNode()->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    rclcpp::QoS qos(1);

    apriltagSub_ =
      this->getNode()
        ->create_subscription<isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray>(
          "/tag_detections", qos,
          std::bind(&ClIsaacApriltag::onAprilTagMessageCallback, this, std::placeholders::_1));
  }

  // subscribe to the apriltag detected
  template <typename T>
  boost::signals2::connection onAprilTagDetected(
    void (T::*callback)(
      const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr &),
    T * object)
  {
    return this->getStateMachine()->createSignalConnection(onAprilTagDetection_, callback, object);
  }

  virtual ~ClIsaacApriltag() {}

  std::vector<std::string> visitedWorkingAreas_;

  std::optional<std::string> selectedVisitTagId_;

  std::map<std::string, geometry_msgs::msg::PoseStamped> detectedAprilTagsMapPose_;

  std::mutex detectedAprilTagsMapPoseMutex_;

  std::map<std::string, geometry_msgs::msg::PoseStamped> getTagsWithinTime(
    rclcpp::Duration duration)
  {
    std::lock_guard<std::mutex> lock(detectedAprilTagsMapPoseMutex_);
    std::map<std::string, geometry_msgs::msg::PoseStamped> ret;

    auto now = getNode()->now();

    std::stringstream ss;
    for (auto & apriltag : detectedAprilTagsMapPose_)
    {
      auto tagstamp = rclcpp::Time(apriltag.second.header.stamp);
      rclcpp::Duration ellapsed = now - tagstamp;
      ;

      ss << "[ClIsaacApriltag] AprilTag: " << apriltag.first << " Ellapsed: " << ellapsed.seconds()
         << " Stamp: " << tagstamp.seconds() << " now " << now.seconds() << std::endl;
      if (ellapsed < duration)
      {
        ret.insert(apriltag);
        ss << "[SELECTD]" << std::endl;
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

private:
  void onAprilTagMessageCallback(
    const isaac_ros_apriltag_interfaces::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(detectedAprilTagsMapPoseMutex_);

    std::stringstream ss;
    for (auto & detection : msg->detections)
    {
      std::string apriltag_frameid = detection.family + ":" + std::to_string(detection.id);
      ss << "[ClIsaacApriltag] AprilTag detected: " << apriltag_frameid << std::endl;

      // get map position using tfListener
      geometry_msgs::msg::TransformStamped transformStampedGlobal;

      try
      {
        transformStampedGlobal =
          tfBuffer_->lookupTransform("map", apriltag_frameid, msg->header.stamp);
      }
      catch (tf2::TransformException & ex)
      {
        RCLCPP_ERROR(getLogger(), "%s", ex.what());
        continue;
      }
      // transform to pose
      geometry_msgs::msg::PoseStamped poseStamped;
      poseStamped.header = transformStampedGlobal.header;
      poseStamped.pose.position.x = transformStampedGlobal.transform.translation.x;
      poseStamped.pose.position.y = transformStampedGlobal.transform.translation.y;
      poseStamped.pose.position.z = transformStampedGlobal.transform.translation.z;
      poseStamped.pose.orientation = transformStampedGlobal.transform.rotation;

      detectedAprilTagsMapPose_[apriltag_frameid] = poseStamped;

      ss << "[ClIsaacApriltag] new AprilTag detected: " << apriltag_frameid << " at "
         << poseStamped.pose.position.x << ", " << poseStamped.pose.position.y << ", "
         << poseStamped.pose.position.z;
    }
    RCLCPP_INFO_STREAM_THROTTLE(getLogger(), *(getNode()->get_clock()), 1000, ss.str());
    onAprilTagDetection_(msg);
  }
};

}  // namespace cl_isaac_apriltag
