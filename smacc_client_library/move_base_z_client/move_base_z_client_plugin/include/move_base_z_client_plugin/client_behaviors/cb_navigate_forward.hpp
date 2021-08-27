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
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <optional>

#include "cb_move_base_client_behavior_base.hpp"

namespace cl_move_base_z
{
class CbNavigateForward : public CbMoveBaseClientBehaviorBase
{
public:
  std::optional<float> forwardDistance;

  // just a stub to show how to use parameterless constructor
  std::optional<float> forwardSpeed;

  // this may be useful in the case you want to do a stright line with some orientation
  // relative to some global reference (trying to solve the initial orientation error
  // of the current orientation). If it is not set, the orientation of the straight line is
  // the orientation of the initial state.
  std::optional<geometry_msgs::msg::Quaternion> forceInitialOrientation;

  std::optional<std::string> goalChecker_;

  std::shared_ptr<tf2_ros::Buffer> listener;

  odom_tracker::OdomTracker * odomTracker_;

  CbNavigateForward(float forwardDistance);

  CbNavigateForward();

  virtual ~CbNavigateForward();

  void onEntry() override;

  void onExit() override;
};
}  // namespace cl_move_base_z
