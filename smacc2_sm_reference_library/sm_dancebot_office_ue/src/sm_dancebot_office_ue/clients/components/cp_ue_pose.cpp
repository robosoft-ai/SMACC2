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
#include <mutex>

#include <geometry_msgs/msg/pose_stamped.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <smacc2/component.hpp>
#include <smacc2/smacc_updatable.hpp>
#include <sm_dancebot_office_ue/clients/components/cp_ue_pose.hpp>

namespace sm_dancebot_office_ue
{
CpUEPose::CpUEPose(std::string topicname)
    : CpTopicSubscriber(topicname) 
{
}

void CpUEPose::onInitialize()
{
    CpTopicSubscriber::onInitialize();
    this->onMessageReceived(&CpUEPose::onPoseMessageReceived, this);    
}

void CpUEPose::onPoseMessageReceived(const ue_msgs::msg::EntityState& msg)
{
    this->entityStateMsg_ = msg;;

    RCLCPP_INFO_STREAM_THROTTLE(getLogger(), *(getNode()->get_clock()), 200, "Received UEPose x: " << msg.pose.position.x << " y: " << msg.pose.position.y << " z: " << msg.pose.position.z);
}

geometry_msgs::msg::Pose CpUEPose::toPoseMsg()
{
    return this->entityStateMsg_.pose;
}

}  // namespace cl_nav2z
