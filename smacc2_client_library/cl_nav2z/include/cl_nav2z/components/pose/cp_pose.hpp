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
 *-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <smacc2/component.hpp>
#include <smacc2/smacc_updatable.hpp>

namespace cl_nav2z
{
enum class StandardReferenceFrames
{
  Map,
  Odometry
};

inline std::string referenceFrameToString(StandardReferenceFrames referenceFrame)
{
  switch (referenceFrame)
  {
    case StandardReferenceFrames::Map:
      return "map";
    case StandardReferenceFrames::Odometry:
      return "odom";
    default:
      return "odom";
  }
}

class Pose : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  Pose(std::string poseFrameName = "base_link", std::string referenceFrame = "odom")
  : isInitialized(false), poseFrameName_(poseFrameName), referenceFrame_(referenceFrame)
  {
    this->pose_.header.frame_id = referenceFrame_;
  }

  Pose(StandardReferenceFrames referenceFrame)
  : Pose("base_link", referenceFrameToString(referenceFrame))
  {
  }

  inline void onInitialize() override
  {
    RCLCPP_INFO(
      getLogger(), "[Pose] Creating Pose tracker component to track %s in the reference frame %s",
      poseFrameName_.c_str(), referenceFrame_.c_str());

    {
      // singleton
      std::lock_guard<std::mutex> guard(listenerMutex_);
      if (tfListener_ == nullptr)
      {
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(getNode()->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
      }
    }
  }

  inline void update() override
  {
    tf2::Stamped<tf2::Transform> transform;
    try
    {
      if (!frozenReferenceFrameTime)
      {
        std::lock_guard<std::mutex> lock(listenerMutex_);
        RCLCPP_INFO(
          getLogger(), "[pose] looking up transform: %s -> %s", referenceFrame_.c_str(),
          poseFrameName_.c_str());
        auto transformstamped =
          tfBuffer_->lookupTransform(referenceFrame_, poseFrameName_, rclcpp::Time(0));
        tf2::fromMsg(transformstamped, transform);
        RCLCPP_INFO(getLogger(), "[pose] transform found");
      }
      else
      {
        RCLCPP_INFO(
          getLogger(), "[pose] looking up transform: %s -> %s", referenceFrame_.c_str(),
          poseFrameName_.c_str());
        auto transformstamped = tfBuffer_->lookupTransform(
          referenceFrame_, *frozenReferenceFrameTime, poseFrameName_, rclcpp::Time(0),
          referenceFrame_, std::chrono::seconds(1));
        tf2::fromMsg(transformstamped, transform);
        RCLCPP_INFO(getLogger(), "[pose] transform found");
      }

      {
        std::lock_guard<std::mutex> guard(m_mutex_);
        tf2::toMsg(transform, this->pose_.pose);
        this->pose_.header.stamp = tf2_ros::toRclcpp(transform.stamp_);
        this->pose_.header.frame_id = referenceFrame_;
        this->isInitialized = true;
      }
    }
    catch (tf2::TransformException & ex)
    {
      // RCLCPP_DEBUG(getLogger(), "[pose] EXCEPTION");
      RCLCPP_ERROR_STREAM_THROTTLE(
        getLogger(), *(getNode()->get_clock()), 1000,
        "[Component pose] (" << poseFrameName_ << "/[" << referenceFrame_
                             << "] ) is failing on pose update : " << ex.what());
    }
  }

  // synchronously waits a transform in the current thread
  inline void waitTransformUpdate(rclcpp::Rate r = rclcpp::Rate(20))
  {
    bool found = false;
    RCLCPP_INFO(getLogger(), "[Pose Component] waitTransformUpdate");
    while (rclcpp::ok() && !found)
    {
      tf2::Stamped<tf2::Transform> transform;
      try
      {
        {
          RCLCPP_INFO_THROTTLE(
            getLogger(), *(getNode()->get_clock()), 1000,
            "[Pose Component] waiting transform %s -> %s", referenceFrame_.c_str(),
            poseFrameName_.c_str());
          std::lock_guard<std::mutex> lock(listenerMutex_);
          auto transformstamped =
            tfBuffer_->lookupTransform(referenceFrame_, poseFrameName_, getNode()->now());
          tf2::fromMsg(transformstamped, transform);
        }

        {
          std::lock_guard<std::mutex> guard(m_mutex_);
          tf2::toMsg(transform, this->pose_.pose);
          this->pose_.header.stamp = tf2_ros::toRclcpp(transform.stamp_);
          this->pose_.header.frame_id = referenceFrame_;
          found = true;
          this->isInitialized = true;
        }
      }
      catch (tf2::TransformException & ex)
      {
        RCLCPP_ERROR_STREAM_THROTTLE(
          getLogger(), *(getNode()->get_clock()), 1000,
          "[Component pose] (" << poseFrameName_ << "/[" << referenceFrame_
                               << "] ) is failing on pose update : " << ex.what());
      }

      r.sleep();
    }
    RCLCPP_INFO(getLogger(), "[Pose Component] waitTransformUpdate -> pose found!");
  }

  inline geometry_msgs::msg::Pose toPoseMsg()
  {
    std::lock_guard<std::mutex> guard(m_mutex_);
    return this->pose_.pose;
  }

  inline geometry_msgs::msg::PoseStamped toPoseStampedMsg()
  {
    RCLCPP_INFO_STREAM(getLogger(), "[Pose] ToPoseMsg ");
    std::lock_guard<std::mutex> guard(m_mutex_);
    return this->pose_;
  }

  // get yaw in radians
  inline float getYaw() { return tf2::getYaw(pose_.pose.orientation); }

  inline float getX() { return pose_.pose.position.x; }
  inline float getY() { return pose_.pose.position.y; }
  inline float getZ() { return pose_.pose.position.z; }

  inline void setReferenceFrame(std::string referenceFrame) { referenceFrame_ = referenceFrame; }

  inline const std::string & getReferenceFrame() const { return referenceFrame_; }

  inline const std::string & getFrameId() const { return poseFrameName_; }

  bool isInitialized;

  std::optional<rclcpp::Time> frozenReferenceFrameTime;
  void freezeReferenceFrame()
  {
    frozenReferenceFrameTime = getNode()->now() - rclcpp::Duration::from_seconds(1);
  }

  void unfreezeReferenceFrame() { frozenReferenceFrameTime = std::nullopt; }

private:
  geometry_msgs::msg::PoseStamped pose_;

  static std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  static std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  static std::mutex listenerMutex_;

  std::string poseFrameName_;
  std::string referenceFrame_;

  std::mutex m_mutex_;
};

// Static member definitions
inline std::shared_ptr<tf2_ros::TransformListener> Pose::tfListener_;
inline std::shared_ptr<tf2_ros::Buffer> Pose::tfBuffer_;
inline std::mutex Pose::listenerMutex_;

}  // namespace cl_nav2z
