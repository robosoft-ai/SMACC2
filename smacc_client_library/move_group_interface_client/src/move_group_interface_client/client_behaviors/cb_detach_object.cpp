/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_detach_object.h>
#include <move_group_interface_client/cl_movegroup.h>

namespace cl_move_group_interface
{
    void CbDetachObject::onEntry()
    {
        cl_move_group_interface::GraspingComponent *graspingComponent;
        this->requiresComponent(graspingComponent);

        cl_move_group_interface::ClMoveGroup *moveGroupClient;
        this->requiresClient(moveGroupClient);

        auto &planningSceneInterface = moveGroupClient->planningSceneInterface;

        moveGroupClient->moveGroupClientInterface.detachObject(*(graspingComponent->currentAttachedObjectName));
        planningSceneInterface.removeCollisionObjects({*(graspingComponent->currentAttachedObjectName)});
    }

    void CbDetachObject::onExit()
    {
    }
} // namespace cl_move_group_interface
