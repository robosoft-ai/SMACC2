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

namespace cl_nav2z
{
class CbRotate : public CbNav2ZClientBehaviorBase
{
public:
  float rotateDegree;

  std::optional<std::string> goalChecker_;

  std::optional<cl_nav2z::SpinningPlanner> spinningPlanner;

  CbRotate(float rotate_degree);

  CbRotate(float rotate_degree, cl_nav2z::SpinningPlanner spinning_planner);

  void onEntry() override;

private:
  //TODO: replace this with the Pose Component in the same way it is done in the CbAbsoluteRotateBehavior
  std::shared_ptr<tf2_ros::Buffer> listener;
};
}  // namespace cl_nav2z
