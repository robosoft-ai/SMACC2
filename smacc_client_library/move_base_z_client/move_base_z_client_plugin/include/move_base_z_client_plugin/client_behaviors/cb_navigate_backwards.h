/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once

#include "cb_move_base_client_behavior_base.h"
#include <tf2_ros/buffer.h>
#include <move_base_z_client_plugin/components/odom_tracker/odom_tracker.h>

namespace cl_move_base_z
{
    // It sends the mobile base some distance backwards
    class CbNavigateBackwards : public CbMoveBaseClientBehaviorBase
    {
    public:
        boost::optional<float> backwardDistance;

        // just a stub to show how to use parameterless constructor
        boost::optional<float> backwardSpeed;

        std::shared_ptr<tf2_ros::Buffer> listener;

        cl_move_base_z::odom_tracker::OdomTracker *odomTracker_;

        CbNavigateBackwards(float backwardDistance);

        CbNavigateBackwards();

        virtual void onEntry() override;

        virtual void onExit() override;
    };
} // namespace cl_move_base_z
