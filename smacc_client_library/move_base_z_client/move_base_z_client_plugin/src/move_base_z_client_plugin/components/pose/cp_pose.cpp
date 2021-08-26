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

#include <move_base_z_client_plugin/components/pose/cp_pose.hpp>

#include <memory>
#include <string>

namespace cl_move_base_z
{
using namespace std::chrono_literals;

// static
std::shared_ptr<tf2_ros::TransformListener> Pose::tfListener_;
std::shared_ptr<tf2_ros::Buffer> Pose::tfBuffer_;
std::mutex Pose::listenerMutex_;

Pose::Pose(std::string targetFrame, std::string referenceFrame)
: isInitialized(false), poseFrameName_(targetFrame), referenceFrame_(referenceFrame)

{
  this->pose_.header.frame_id = referenceFrame_;
}

void Pose::onInitialize()
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

void Pose::waitTransformUpdate(rclcpp::Rate r)
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

void Pose::update()
{
  tf2::Stamped<tf2::Transform> transform;
  try
  {
    {
      std::lock_guard<std::mutex> lock(listenerMutex_);
      RCLCPP_DEBUG(getLogger(), "[pose] looking up transform");
      auto transformstamped =
        tfBuffer_->lookupTransform(referenceFrame_, poseFrameName_, rclcpp::Time(0));
      tf2::fromMsg(transformstamped, transform);
    }

    {
      std::lock_guard<std::mutex> guard(m_mutex_);
      tf2::toMsg(transform, this->pose_.pose);
      this->pose_.header.stamp = tf2_ros::toRclcpp(transform.stamp_);
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
}  // namespace cl_move_base_z
