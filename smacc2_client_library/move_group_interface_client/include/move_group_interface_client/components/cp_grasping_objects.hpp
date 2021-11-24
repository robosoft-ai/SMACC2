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
 *****************************************************************************************************************/

#pragma once

#include <map>
#include <moveit_msgs/msg/collision_object.hpp>
#include <smacc2/component.hpp>
#include <smacc2/smacc.hpp>

namespace cl_move_group_interface
{
class CpGraspingComponent : public smacc2::ISmaccComponent
{
private:
  std::map<std::string, moveit_msgs::msg::CollisionObject> graspingObjects;

public:
  std::vector<std::string> fingerTipNames;
  std::string gripperLink_ = "gripper_link";

  std::optional<std::string> currentAttachedObjectName;

  bool getGraspingObject(std::string name, moveit_msgs::msg::CollisionObject & object);

  void createGraspableBox(
    std::string frameid, float x, float y, float z, float xl, float yl, float zl);
};

}  // namespace cl_move_group_interface
