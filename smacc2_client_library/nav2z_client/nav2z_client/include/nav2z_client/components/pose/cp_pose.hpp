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

#include <mutex>

#include <geometry_msgs/msg/pose_stamped.h>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <smacc2/component.hpp>
#include <smacc2/smacc_updatable.hpp>

namespace cl_nav2z
{
enum class StandardReferenceFrames
{
  Map,
  Odometry
};

class Pose : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  Pose(std::string poseFrameName = "base_link", std::string referenceFrame = "odom");

  Pose(StandardReferenceFrames referenceFrame);

  void onInitialize() override;

  void update() override;

  // synchronously waits a transform in the current thread
  void waitTransformUpdate(rclcpp::Rate r = rclcpp::Rate(20));

  inline geometry_msgs::msg::Pose toPoseMsg()
  {
    std::lock_guard<std::mutex> guard(m_mutex_);
    return this->pose_.pose;
  }

  inline geometry_msgs::msg::PoseStamped toPoseStampedMsg()
  {
    std::lock_guard<std::mutex> guard(m_mutex_);
    return this->pose_;
  }

  // get yaw in radians
  float getYaw();

  float getX();
  float getY();
  float getZ();

  inline const std::string & getReferenceFrame() const { return referenceFrame_; }

  inline const std::string & getFrameId() const { return poseFrameName_; }

  bool isInitialized;

private:
  geometry_msgs::msg::PoseStamped pose_;

  static std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  static std::shared_ptr<tf2_ros::TransformListener> tfListener_;

  static std::mutex listenerMutex_;

  std::string poseFrameName_;
  std::string referenceFrame_;

  std::mutex m_mutex_;
};
}  // namespace cl_nav2z
