/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include "cb_move_base_client_behavior_base.h"
#include <move_base_z_client_plugin/move_base_z_client_plugin.h>

namespace cl_move_base_z
{
    class CbNavigateNextWaypoint : public CbMoveBaseClientBehaviorBase
    {
    public:
        CbNavigateNextWaypoint();
        
        virtual ~CbNavigateNextWaypoint();

        virtual void onEntry() override;

        virtual void onExit() override;
    };
} // namespace cl_move_base_z
