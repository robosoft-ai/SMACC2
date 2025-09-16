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

#include "cb_nav2z_client_behavior_base.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cl_nav2z
{
class CbAbsoluteRotate : public CbNav2ZClientBehaviorBase
{
public:
  std::shared_ptr<tf2_ros::Buffer> listener;

  float absoluteGoalAngleDegree;
  std::optional<float> yawGoalTolerance;
  std::optional<float> maxVelTheta;  // if not defined, default parameter server
  std::optional<SpinningPlanner> spinningPlanner;

  CbAbsoluteRotate();
  CbAbsoluteRotate(float absoluteGoalAngleDegree, float yawGoalTolerance = -1);

  void onEntry() override;
  void onExit() override;

private:
  void updateTemporalBehaviorParameters(bool undo);
  float oldYawTolerance;
  float oldMaxVelTheta;
  float oldMinVelTheta;
};
}  // namespace cl_nav2z
