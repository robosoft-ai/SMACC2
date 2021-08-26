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

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <smacc/component.hpp>
#include <smacc/smacc_updatable.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cl_move_base_z
{
class Pose : public smacc::ISmaccComponent, public smacc::ISmaccUpdatable
{
public:
  Pose(std::string poseFrameName = "base_link", std::string referenceFrame = "odom");

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
}  // namespace cl_move_base_z