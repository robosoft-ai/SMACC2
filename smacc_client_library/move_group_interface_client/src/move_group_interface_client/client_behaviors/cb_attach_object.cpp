/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_attach_object.h>
#include <move_group_interface_client/components/cp_grasping_objects.h>

namespace cl_move_group_interface
{
    CbAttachObject::CbAttachObject(std::string targetObjectName)
        : targetObjectName_(targetObjectName)
    {
    }

    CbAttachObject::CbAttachObject()
    {
        }

    void CbAttachObject::onEntry()
    {
        cl_move_group_interface::ClMoveGroup *moveGroup;
        this->requiresClient(moveGroup);

        cl_move_group_interface::GraspingComponent *graspingComponent;
        this->requiresComponent(graspingComponent);

        // auto cubepos = cubeinfo->pose_->toPoseStampedMsg();

        moveit_msgs::msgs::CollisionObject targetCollisionObject;

        bool found = graspingComponent->getGraspingObject(targetObjectName_, targetCollisionObject);

        if (found)
        {
            targetCollisionObject.operation = moveit_msgs::msgs::CollisionObject::ADD;
            targetCollisionObject.header.stamp = nh_->now();();

            moveGroup->planningSceneInterface.applyCollisionObject(targetCollisionObject);
            // collisionObjects.push_back(cubeCollision);

            graspingComponent->currentAttachedObjectName = targetObjectName_;
            moveGroup->moveGroupClientInterface.attachObject(targetObjectName_, "gripper_link", graspingComponent->fingerTipNames);
        }
    }

    void CbAttachObject::onExit()
    {
    }
} // namespace cl_move_group_interface