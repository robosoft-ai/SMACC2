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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "cb_absolute_rotate.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cl_nav2z
{
class CbRotateLookAt : public CbAbsoluteRotate
{
public:
  std::shared_ptr<tf2_ros::Buffer> listener;

  std::optional<geometry_msgs::msg::PoseStamped> lookAtPose_;

  CbRotateLookAt();
  CbRotateLookAt(const geometry_msgs::msg::PoseStamped & lookAtPose);

  void onEntry() override;
  // void onExit() override;
};
}  // namespace cl_nav2z
