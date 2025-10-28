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
#include <smacc2/smacc.hpp>

namespace cl_moveit2z
{
/**
 * @brief Client behavior that detaches the currently attached collision object
 *
 * This behavior detaches whatever object is currently attached to the robot's
 * gripper (tracked in CpGraspingComponent::currentAttachedObjectName) and
 * removes it from the planning scene. Posts success or failure event based
 * on the detach operation result.
 */
class CbDetachObject : public smacc2::SmaccAsyncClientBehavior
{
public:
  /**
   * @brief Called when the behavior is entered
   *
   * Retrieves the currently attached object name from CpGraspingComponent,
   * detaches it from the gripper, and removes it from the planning scene.
   */
  inline void onEntry() override
  {
    cl_moveit2z::CpGraspingComponent * graspingComponent;
    this->requiresComponent(graspingComponent);

    cl_moveit2z::ClMoveit2z * moveGroupClient;
    this->requiresClient(moveGroupClient);

    if (graspingComponent->currentAttachedObjectName)
    {
      RCLCPP_INFO_STREAM(
        getLogger(),
        "[CbDetachObject] Detaching object: " << *(graspingComponent->currentAttachedObjectName));

      auto & planningSceneInterface = moveGroupClient->planningSceneInterface;
      auto res = moveGroupClient->moveGroupClientInterface->detachObject(
        *(graspingComponent->currentAttachedObjectName));

      planningSceneInterface->removeCollisionObjects(
        {*(graspingComponent->currentAttachedObjectName)});

      if (res)
      {
        RCLCPP_INFO(getLogger(), "[CbDetachObject] Detach succeeded");
        this->postSuccessEvent();
      }
      else
      {
        RCLCPP_ERROR(getLogger(), "[CbDetachObject] Detach failed");
        this->postFailureEvent();
      }
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "[CbDetachObject] No object currently attached");
      this->postFailureEvent();
    }
  }

  /**
   * @brief Called when the behavior is exited
   */
  inline void onExit() override {}
};

}  // namespace cl_moveit2z
