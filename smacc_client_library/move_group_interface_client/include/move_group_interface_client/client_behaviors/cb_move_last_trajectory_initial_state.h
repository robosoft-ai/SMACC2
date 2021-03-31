/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include "cb_move_joints.h"

namespace cl_move_group_interface
{
    class CbMoveLastTrajectoryInitialState : public CbMoveJoints
    {
    public:
        CbMoveLastTrajectoryInitialState();

        CbMoveLastTrajectoryInitialState(int backIndex);

        virtual ~CbMoveLastTrajectoryInitialState();

        virtual void onEntry() override;

    private:
        int backIndex_ = -1;
    };
} // namespace cl_move_group_interface