// Copyright 2025 Robosoft Inc.
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
#include <cl_moveit2z/client_behaviors/cb_moveit2z_client_behavior_base.hpp>
#include <cl_moveit2z/components/cp_grasping_objects.hpp>
#include <cl_moveit2z/components/cp_move_group_interface.hpp>
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
class CbDetachObject : public CbMoveit2zClientBehaviorBase
{
public:
  /**
   * @brief Called when the behavior is entered
   *
   * Retrieves the currently attached object name from CpGraspingComponent,
   * detaches it from the gripper, and removes it from the planning scene.
   */
  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    this->requiresComponent(graspingComponent_, smacc2::ComponentRequirement::HARD);
    CbMoveit2zClientBehaviorBase::onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

  inline void onEntry() override
  {
    // components resolved in onStateOrthogonalAllocation
    auto * graspingComponent = graspingComponent_;
    auto * cpMoveGroup = cpMoveGroup_;

    if (graspingComponent->currentAttachedObjectName)
    {
      RCLCPP_INFO_STREAM(
        getLogger(),
        "[CbDetachObject] Detaching object: " << *(graspingComponent->currentAttachedObjectName));

      auto & planningSceneInterface = cpMoveGroup->planningSceneInterface;
      auto res = cpMoveGroup->moveGroupClientInterface->detachObject(
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

private:
  cl_moveit2z::CpGraspingComponent * graspingComponent_ = nullptr;
};

}  // namespace cl_moveit2z
