/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/client_behaviors/cb_move_last_trajectory_initial_state.h>
#include <move_group_interface_client/components/cp_trajectory_history.h>

namespace cl_move_group_interface
{
    CbMoveLastTrajectoryInitialState::CbMoveLastTrajectoryInitialState()
    {
    }

    CbMoveLastTrajectoryInitialState::CbMoveLastTrajectoryInitialState(int backIndex)
        : backIndex_(backIndex)
    {
    }

    CbMoveLastTrajectoryInitialState::~CbMoveLastTrajectoryInitialState()
    {
    }

    void CbMoveLastTrajectoryInitialState::onEntry()
    {
        CpTrajectoryHistory *trajectoryHistory;
        this->requiresComponent(trajectoryHistory);

        if (trajectoryHistory != nullptr)
        {
            moveit_msgs::RobotTrajectory trajectory;

            bool trajectoryFound = trajectoryHistory->getLastTrajectory(backIndex_, trajectory);

            if (trajectoryFound)
            {
                trajectory_msgs::JointTrajectoryPoint &initialPoint = trajectory.joint_trajectory.points.front();

                std::stringstream ss;
                for (int i = 0; i < trajectory.joint_trajectory.joint_names.size(); i++)
                {
                    auto &name = trajectory.joint_trajectory.joint_names[i];

                    jointValueTarget_[name] = initialPoint.positions[i];
                    ss << name << ": " << jointValueTarget_[name] << std::endl;
                }
                RCLCPP_INFO_STREAM(nh_->get_logger(), "[" << this->getName() << "]" << std::endl
                                    << ss.str());

                RCLCPP_INFO_STREAM(nh_->get_logger(), "[" << this->getName() << "] move joint onEntry");
                CbMoveJoints::onEntry();
                RCLCPP_INFO_STREAM(nh_->get_logger(), "[" << this->getName() << "] move joint onEntry finished");
            }
        }

        //call base OnEntry
    }
} // namespace cl_move_group_interface