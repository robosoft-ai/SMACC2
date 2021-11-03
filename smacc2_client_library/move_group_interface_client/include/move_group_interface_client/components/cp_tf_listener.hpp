/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <smacc2/component.h>
#include <smacc2/smacc_updatable.h>

#include <tf2/transform_datatypes.h>
#include <tf2/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <thread>

namespace cl_move_base_z
{
struct TfPoseTrack
{
  std::mutex mutex_;
  geometry_msgs::msg::PoseStamped pose_;
  std::string targetPoseFrame_;
  std::string referenceBaseFrame_;
  bool once = true;
  bool isInitialized = false;
};

class CpTFListener : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  CpTFListener();

  virtual void update() override
  {
    for (auto & poseTrack : poseTracks_)
    {
      tf2::Stamped<tf2::Transform> transform;
      try
      {
        {
          std::lock_guard<std::mutex> lock(listenerMutex_);
          tfListener_->lookupTransform(
            poseTrack->referenceBaseFrame_, poseTrack->targetPoseFrame_, rclcpp::Time(0),
            transform);
        }

        {
          std::lock_guard<std::mutex> guard(m_mutex_);
          poseTrack->pose_.pose = tf2::toMsg(transform);
          poseTrack->pose_.header.stamp = transform.stamp_;
          poseTrack->pose_.header.frame_id = poseTrack->referenceBaseFrame_;
          poseTrack->isInitialized = true;
        }
      }
      catch (tf2::TransformException ex)
      {
        RCLCPP_ERROR_STREAM_THROTTLE(
          1, "[Component pose] (getLogger(), " << poseFrameName_ << "/[" << referenceFrame_
                                               << "] ) is failing on pose update : " << ex.what());
      }
    }
  }

  void getLastTransform(
    std::string & targetPoseFrameName, std::string & referenceBaseFrame,
    geometry_msgs::msg::Pose & out)
  {
  }

  std::future<geometry_msgs::msg::Pose> waitForNextTransform(
    std::string & targetName, std::string & referenceBaseFrame)
  {
    tracks_
  }

private:
  static std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  static std::mutex listenerMutex_;

  std::mutex m_mutex_;
  std::list<std::shared_ptr<TfPoseTrack>> poseTracks_;
};
}  // namespace cl_move_base_z
