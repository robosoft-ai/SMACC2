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

#pragma once
#include <smacc2/component.hpp>
#include <smacc2/smacc_updatable.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cl_foundation_pose/components/tracker_utils.hpp>

namespace cl_foundation_pose
{

using namespace smacc2::client_core_components;


class CpObjectTrackerTf : public smacc2::ISmaccComponent , public smacc2::ISmaccUpdatable
{

private:
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

  std::string global_frame_id_;
  bool enabled_= false;

  // Declare a data structure to store the detected objects.
  std::map<std::string, DetectedObject> detectedObjects;
  std::mutex m_mutex_;

public:
CpObjectTrackerTf(std::string global_frame_id="map") : global_frame_id_(global_frame_id) {}

  void setEnabled(bool enabled)
  {
    resetPoseEstimation();
    enabled_ = enabled;
  }

  bool isEnabled()
  {
    return enabled_;
  }

  void resetPoseEstimation()
  {
    std::lock_guard<std::mutex> lock(m_mutex_);

    RCLCPP_INFO(getLogger(), "CpObjectTrackerTf::resetPoseEstimation() tracked objects: %ld", detectedObjects.size());
    for(auto &detectedObject : detectedObjects)
    {
      auto& detectedObjectInfo = detectedObject.second;
      RCLCPP_INFO(getLogger(), "CpObjectTrackerTf::resetPoseEstimation() tracking object: %s", detectedObject.first.c_str());

      detectedObjectInfo.filtered_pose.reset();
      detectedObjectInfo.historicalPoses_.clear();
    }
    detectedObjects.clear();

  }

  void update() override
  {
    RCLCPP_INFO(getLogger(), "CpObjectTrackerTf::update() tracked objects: %ld", detectedObjects.size());
    std::lock_guard<std::mutex> lock(m_mutex_);

    if(!enabled_)
      return;

    RCLCPP_INFO(getLogger(), "CpObjectTrackerTf::update() heartbeat, tracked objects: %ld", detectedObjects.size());

    // refresh tracked object poses
    for (auto &detectedObject : detectedObjects)
    {
      RCLCPP_INFO(getLogger(), "CpObjectTrackerTf::update() tracking object: %s", detectedObject.first.c_str());

      auto globalObjectPose = this->updateAndGetGlobalPose(detectedObject.first, global_frame_id_);
      if (globalObjectPose)
      {
        detectedObject.second.filtered_pose = *globalObjectPose;
      }
    }
  }


  void onInitialize()
  {
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->getNode()->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->getNode());
  }

  std::optional<geometry_msgs::msg::PoseStamped> updateAndGetGlobalPose(const std::string& child_frame_id, const std::string& frame_id)
  {
    RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] updateAndGetGlobalPose('%s', '%s')", child_frame_id.c_str(), frame_id.c_str());
    // check in database if the object is already tracked
    auto object = detectedObjects.find(child_frame_id);
    DetectedObject* detectedObject = nullptr;
    if (object != detectedObjects.end()) // already tracked
    {
      detectedObject = &object->second;
      RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] tracking existing object: %s", child_frame_id.c_str());
    }
    else
    {
      detectedObjects[child_frame_id] = DetectedObject();
      detectedObject = &detectedObjects[child_frame_id];
      RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] tracking new object: %s", child_frame_id.c_str());
    }

    if (tfBuffer_->canTransform(frame_id, child_frame_id, rclcpp::Time(0)))
    {
         geometry_msgs::msg::PoseStamped pose;

         auto transformStamped = tfBuffer_->lookupTransform(frame_id, child_frame_id, rclcpp::Time(0));
         pose.header = transformStamped.header;
         pose.pose.position.x = transformStamped.transform.translation.x;
         pose.pose.position.y = transformStamped.transform.translation.y;
         pose.pose.position.z = transformStamped.transform.translation.z;
         pose.pose.orientation = transformStamped.transform.rotation;

         RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] *updateAndGetGlobalPose('%s', '%s') pose: %f, %f, %f", child_frame_id.c_str(), frame_id.c_str(), pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));

        auto& historicalPoses_ = detectedObject->historicalPoses_;
        RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] updateAndGetGlobalPose('%s', '%s') historical poses: %ld", child_frame_id.c_str(), frame_id.c_str(), historicalPoses_.size() );

         const size_t MAX_HISTORY=512;
         if(historicalPoses_.size() > MAX_HISTORY)
         {
           RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] updateAndGetGlobalPose('%s', '%s') historical poses, popping oldest pose", child_frame_id.c_str(), frame_id.c_str());
           historicalPoses_.erase(historicalPoses_.begin());
         }
         RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] updateAndGetGlobalPose('%s', '%s') historical poses, pushing new pose", child_frame_id.c_str(), frame_id.c_str());
         historicalPoses_.push_back(pose);

         RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] updateAndGetGlobalPose('%s', '%s') -presort- historical poses: %ld", child_frame_id.c_str(), frame_id.c_str(), historicalPoses_.size() );

         // compute median position in x
         std::sort(historicalPoses_.begin(), historicalPoses_.end(), [](const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b) {
           return a.pose.position.x < b.pose.position.x;
         });


         RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] updateAndGetGlobalPose('%s', '%s')- presort- historical poses: %ld", child_frame_id.c_str(), frame_id.c_str(), historicalPoses_.size() );

         geometry_msgs::msg::PoseStamped medianPose;
         medianPose.pose.position.x = historicalPoses_[historicalPoses_.size()/2].pose.position.x;

         // compute median position in y
         std::sort(historicalPoses_.begin(), historicalPoses_.end(), [](const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b) {
           return a.pose.position.y < b.pose.position.y;
         });

         medianPose.pose.position.y = historicalPoses_[historicalPoses_.size()/2].pose.position.y;
         medianPose.header = pose.header;

         detectedObject->filtered_pose= medianPose;


         RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] updateAndGetGlobalPose('%s', '%s') filtered pose [%ld samples]: %f, %f, %f", child_frame_id.c_str(), frame_id.c_str(), detectedObject->historicalPoses_.size() , detectedObject->filtered_pose->pose.position.x, detectedObject->filtered_pose->pose.position.y,tf2::getYaw(detectedObject->filtered_pose->pose.orientation));
         this->postEvent<EvObjectDetected>();

         return detectedObject->filtered_pose;
    }

    RCLCPP_INFO(getLogger(), "[CpObjectTrackerTf] updateAndGetGlobalPose('%s', '%s') failed", child_frame_id.c_str(), frame_id.c_str());

    return std::nullopt;
  }

};

} // namespace cl_foundation_pose
