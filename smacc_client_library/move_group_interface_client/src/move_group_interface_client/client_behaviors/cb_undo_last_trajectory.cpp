/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <move_group_interface_client/client_behaviors/cb_undo_last_trajectory.h>
#include <move_group_interface_client/components/cp_trajectory_history.h>

namespace cl_move_group_interface
{

    CbUndoLastTrajectory::CbUndoLastTrajectory()
    {
    }

    CbUndoLastTrajectory::CbUndoLastTrajectory(int backIndex)
        : backIndex_(backIndex)
    {
    }

    CbUndoLastTrajectory::~CbUndoLastTrajectory()
    {
    }

    void CbUndoLastTrajectory::onEntry()
    {
        CpTrajectoryHistory *trajectoryHistory;
        this->requiresComponent(trajectoryHistory);
        this->requiresClient(movegroupClient_);

        if (trajectoryHistory->getLastTrajectory(backIndex_, trajectory))
        {
            auto initialTime = trajectory.joint_trajectory.points.back().time_from_start;

            reversed = trajectory;

            std::reverse(reversed.joint_trajectory.points.begin(), reversed.joint_trajectory.points.end());

            for (auto &jp : reversed.joint_trajectory.points)
            {
                jp.time_from_start = ros::Duration(fabs(jp.time_from_start.toSec() - initialTime.toSec())); //ros::Duration(t);
            }

            this->executeJointSpaceTrajectory(reversed);
        }
    }

} // namespace cl_move_group_interface