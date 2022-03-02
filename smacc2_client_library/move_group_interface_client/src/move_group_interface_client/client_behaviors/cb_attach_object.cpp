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

#include <move_group_interface_client/client_behaviors/cb_attach_object.hpp>
#include <move_group_interface_client/components/cp_grasping_objects.hpp>

namespace cl_move_group_interface
{
CbAttachObject::CbAttachObject(std::string targetObjectName) : targetObjectName_(targetObjectName)
{
}

CbAttachObject::CbAttachObject() {}

void CbAttachObject::onEntry()
{
  cl_move_group_interface::ClMoveGroup * moveGroup;
  this->requiresClient(moveGroup);

  cl_move_group_interface::CpGraspingComponent * graspingComponent;
  this->requiresComponent(graspingComponent);

  // auto cubepos = cubeinfo->pose_->toPoseStampedMsg();

  moveit_msgs::msg::CollisionObject targetCollisionObject;

  bool found = graspingComponent->getGraspingObject(targetObjectName_, targetCollisionObject);

  if (found)
  {
    targetCollisionObject.operation = moveit_msgs::msg::CollisionObject::ADD;
    targetCollisionObject.header.stamp = getNode()->now();

    moveGroup->planningSceneInterface->applyCollisionObject(targetCollisionObject);
    // collisionObjects.push_back(cubeCollision);

    graspingComponent->currentAttachedObjectName = targetObjectName_;
    moveGroup->moveGroupClientInterface->attachObject(
      targetObjectName_, graspingComponent->gripperLink_, graspingComponent->fingerTipNames);

    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] Grasping objectfound. attach request.");

    this->postSuccessEvent();
  }
  else
  {
    RCLCPP_WARN_STREAM(
      getLogger(), "[" << getName() << "] Grasping object was not found. Ignoring attach request.");

    this->postFailureEvent();
  }
}

void CbAttachObject::onExit() {}
}  // namespace cl_move_group_interface
