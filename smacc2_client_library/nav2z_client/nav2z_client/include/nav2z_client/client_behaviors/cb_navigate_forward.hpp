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

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2z_client/components/odom_tracker/odom_tracker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <optional>

#include "cb_nav2z_client_behavior_base.hpp"

namespace cl_nav2z
{
class CbNavigateForward : public CbNav2ZClientBehaviorBase
{
public:
  // just a stub to show how to use parameterless constructor
  std::optional<float> forwardSpeed;

  // this may be useful in the case you want to do a straight line with some known direction
  // and the robot may not have that specific initial orientation at that moment.
  // If it is not set, the orientation of the straight line is the orientation of the initial (current) state.
  std::optional<geometry_msgs::msg::Quaternion> forceInitialOrientation;

  // the name of the goal checker selected in the navigation2 stack
  std::optional<std::string> goalChecker_;

  CbNavigateForward();

  CbNavigateForward(float forwardDistance);

  // alternative forward motion using an absolute goal position instead a relative linear distance.
  // It is specially to retry a previous relative incorrect motions based on distance.
  // The developer user is in charge of setting a valid goal position that is located forward
  // from the current position and orientation.
  CbNavigateForward(geometry_msgs::msg::PoseStamped goalPosition);

  virtual ~CbNavigateForward();

  void onEntry() override;

  void onExit() override;

  void setForwardDistance(float distance_meters);

protected:
  // required component
  odom_tracker::OdomTracker * odomTracker_;

  std::optional<geometry_msgs::msg::PoseStamped> goalPose_;

  std::optional<float> forwardDistance_;
};
}  // namespace cl_nav2z
