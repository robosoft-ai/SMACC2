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

#include <move_group_interface_client/components/cp_grasping_objects.hpp>

namespace cl_move_group_interface
{
bool CpGraspingComponent::getGraspingObject(
  std::string name, moveit_msgs::msg::CollisionObject & object)
{
  if (this->graspingObjects.count(name))
  {
    object = this->graspingObjects[name];
    return true;
  }
  else
  {
    return false;
  }
}

void CpGraspingComponent::createGraspableBox(
  std::string frameid, float x, float y, float z, float xl, float yl, float zl)
{
  RCLCPP_INFO_STREAM(
    getLogger(), "[" << getName() << "] creating grasping object in planning scene: " << frameid);
  moveit_msgs::msg::CollisionObject collision;
  auto boxname = frameid;
  ;
  collision.id = boxname;
  collision.header.frame_id = frameid;

  collision.primitives.resize(1);
  collision.primitives[0].type = collision.primitives[0].BOX;
  collision.primitives[0].dimensions.resize(3);

  collision.primitives[0].dimensions[0] = xl;
  collision.primitives[0].dimensions[1] = yl;
  collision.primitives[0].dimensions[2] = zl;

  collision.primitive_poses.resize(1);
  collision.primitive_poses[0].position.x = x;
  collision.primitive_poses[0].position.y = y;
  collision.primitive_poses[0].position.z = z;
  collision.primitive_poses[0].orientation.w = 1.0;

  graspingObjects[boxname] = collision;
}
}  // namespace cl_move_group_interface
