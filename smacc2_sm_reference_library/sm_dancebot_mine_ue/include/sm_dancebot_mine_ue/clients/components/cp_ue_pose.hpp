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
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <smacc2/component.hpp>
#include <smacc2/smacc_updatable.hpp>
#include <ue_msgs/msg/entity_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <smacc2/client_base_components/cp_topic_subscriber.hpp>

namespace sm_dancebot_mine_ue
{

class CpUEPose : public smacc2::components::CpTopicSubscriber<ue_msgs::msg::EntityState>
{
public:
  CpUEPose(std::string topicname);

  void onInitialize() override;
  
  void onPoseMessageReceived(const ue_msgs::msg::EntityState& msg);

  geometry_msgs::msg::Pose toPoseMsg();

private:
  ue_msgs::msg::EntityState entityStateMsg_;

};
}  // namespace sm_dancebot_mine_ue
