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

#include <cl_moveit2z/cl_moveit2z.hpp>
#include <cl_moveit2z/components/cp_grasping_objects.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <smacc2/smacc.hpp>

namespace cl_moveit2z
{
/**
 * @brief Client behavior that attaches a collision object to the robot gripper
 *
 * This behavior attaches a previously created grasping object to the robot's
 * end effector. The object must exist in the CpGraspingComponent's graspingObjects
 * map. Upon successful attachment, the object is added to the planning scene
 * and attached to the gripper using the finger tip links.
 */
class CbAttachObject : public smacc2::SmaccAsyncClientBehavior
{
public:
  /**
   * @brief Constructor with object name
   * @param targetObjectName Name of the object to attach (must exist in CpGraspingComponent)
   */
  inline CbAttachObject(std::string targetObjectName) : targetObjectName_(targetObjectName) {}

  /**
   * @brief Default constructor
   */
  inline CbAttachObject() {}

  /**
   * @brief Called when the behavior is entered
   *
   * Retrieves the target object from CpGraspingComponent, adds it to the planning
   * scene, and attaches it to the gripper. Posts success or failure event based
   * on whether the object was found.
   */
  inline void onEntry() override
  {
    cl_moveit2z::ClMoveit2z * moveGroup;
    this->requiresClient(moveGroup);

    cl_moveit2z::CpGraspingComponent * graspingComponent;
    this->requiresComponent(graspingComponent);

    moveit_msgs::msg::CollisionObject targetCollisionObject;
    bool found = graspingComponent->getGraspingObject(targetObjectName_, targetCollisionObject);

    if (found)
    {
      RCLCPP_INFO_STREAM(getLogger(), "[CbAttachObject] Attaching object: " << targetObjectName_);

      targetCollisionObject.operation = moveit_msgs::msg::CollisionObject::ADD;
      targetCollisionObject.header.stamp = getNode()->now();

      moveGroup->planningSceneInterface->applyCollisionObject(targetCollisionObject);
      graspingComponent->currentAttachedObjectName = targetObjectName_;

      moveGroup->moveGroupClientInterface->attachObject(
        targetObjectName_, graspingComponent->gripperLink_, graspingComponent->fingerTipNames);

      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_ERROR_STREAM(
        getLogger(),
        "[CbAttachObject] Object not found in grasping component: " << targetObjectName_);
      this->postFailureEvent();
    }
  }

  /**
   * @brief Called when the behavior is exited
   */
  inline void onExit() override {}

  /// Name of the target object to attach
  std::string targetObjectName_;
};

}  // namespace cl_moveit2z
