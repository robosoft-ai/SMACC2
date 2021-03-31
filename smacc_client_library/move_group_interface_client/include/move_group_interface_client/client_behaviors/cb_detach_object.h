/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/smacc.h>
#include <move_group_interface_client/components/cp_grasping_objects.h>
#include <move_group_interface_client/cl_movegroup.h>

namespace cl_move_group_interface
{
    class CbDetachObject : public smacc::SmaccClientBehavior
    {
    public:
        virtual void onEntry() override;

        virtual void onExit() override;
    };
} // namespace cl_move_group_interface