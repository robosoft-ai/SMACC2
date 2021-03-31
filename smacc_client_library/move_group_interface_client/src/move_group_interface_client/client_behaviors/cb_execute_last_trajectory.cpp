/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <move_group_interface_client/client_behaviors/cb_execute_last_trajectory.h>
#include <move_group_interface_client/components/cp_trajectory_history.h>

namespace cl_move_group_interface
{
    CbExecuteLastTrajectory::CbExecuteLastTrajectory()
    {
    }

    CbExecuteLastTrajectory::~CbExecuteLastTrajectory()
    {

    }

    void CbExecuteLastTrajectory::onEntry()
    {
        this->requiresClient(movegroupClient_);

        CpTrajectoryHistory *trajectoryHistory;
        this->requiresComponent(trajectoryHistory);

        // this->generateTrajectory();
        // endEffectorTrajectory_ =

        // if (this->endEffectorTrajectory_.size() == 0)
        // {
        //     RCLCPP_WARN_STREAM(nh_->get_logger(), "[" << smacc::demangleSymbol(typeid(*this).name()) << "] No points in the trajectory. Skipping behavior.");
        //     return;
        // }

        //this->createMarkers();
        //markersInitialized_ = true;

        moveit_msgs::RobotTrajectory trajectory;

        if (trajectoryHistory->getLastTrajectory(trajectory))
        {
            this->executeJointSpaceTrajectory(trajectory);
        }
    }

} // namespace cl_move_group_interface