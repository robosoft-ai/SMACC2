/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include <move_group_interface_client/client_behaviors/cb_move_end_effector_trajectory.h>
#include <move_group_interface_client/components/cp_trajectory_history.h>

namespace cl_move_group_interface
{
    class CbUndoLastTrajectory : public CbMoveEndEffectorTrajectory
    {
    public:
        CbUndoLastTrajectory();

        CbUndoLastTrajectory(int backIndex);

        virtual ~CbUndoLastTrajectory();

        virtual void onEntry() override;
    private:
        int backIndex_ = -1;

        moveit_msgs::msgs::RobotTrajectory trajectory;
        moveit_msgs::msgs::RobotTrajectory reversed;
    };

} // namespace cl_move_group_interface