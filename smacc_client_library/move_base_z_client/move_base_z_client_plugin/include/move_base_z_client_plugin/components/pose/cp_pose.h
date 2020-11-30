/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc/component.h>
#include <smacc/smacc_updatable.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose.hpp>
#include <mutex>

namespace cl_move_base_z
{
class Pose : public smacc::ISmaccComponent, public smacc::ISmaccUpdatable
{
public:
  Pose(std::string poseFrameName = "base_link", std::string referenceFrame = "odom");

  virtual void onInitialize() override;

  virtual void update() override;

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

  inline const std::string &getReferenceFrame() const
  {
    return referenceFrame_;
  }

  inline const std::string &getFrameId() const
  {
    return poseFrameName_;
  }

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
