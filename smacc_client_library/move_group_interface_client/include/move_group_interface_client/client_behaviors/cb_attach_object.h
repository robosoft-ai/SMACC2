/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/smacc.h>
#include <move_group_interface_client/cl_movegroup.h>

namespace cl_move_group_interface
{
    class CbAttachObject : public smacc::SmaccClientBehavior
    {
    public:
        CbAttachObject(std::string targetObjectName);

        CbAttachObject();

        virtual void onEntry() override;

        virtual void onExit() override;

        std::string targetObjectName_;

    private:
    };
} // namespace cl_move_group_interface