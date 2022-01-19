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

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "cb_nav2z_client_behavior_base.hpp"

namespace cl_nav2z
{
class CbNavigateGlobalPosition : public CbNav2ZClientBehaviorBase
{
public:
  geometry_msgs::msg::Point goalPosition;
  float goalYaw;

  std::optional<float> yawTolerance;
  std::optional<float> yawToleranceX;
  std::optional<float> yawToleranceY;

  std::optional<std::string> goalChecker_;

  CbNavigateGlobalPosition();

  CbNavigateGlobalPosition(float x, float y, float yaw /*radians*/);

  void setGoal(const geometry_msgs::msg::Pose & pose);

  virtual void onEntry() override;

  // This is the substate destructor. This code will be executed when the
  // workflow exits from this substate (that is according to statechart the moment when this object is destroyed)
  void onExit() override;

  // auxiliary function that defines the motion that is requested to the navigation2 action server
  void execute();

private:
  void readStartPoseFromParameterServer(ClNav2Z::Goal & goal);
};
}  // namespace cl_nav2z
