/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2020
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <move_group_interface_client/components/cp_trajectory_history.h>

namespace cl_move_group_interface
{

    bool CpTrajectoryHistory::getLastTrajectory(int backIndex, moveit_msgs::RobotTrajectory &trajectory)
    {
        if (trajectoryHistory_.size() == 0 )
        {
            return false;
        }

        if (backIndex < 0)
        {
            backIndex = 0;
        }
        else if(backIndex >= this->trajectoryHistory_.size())
        {
            return false; 
        }

        trajectory = this->trajectoryHistory_[this->trajectoryHistory_.size() - 1 - backIndex].trajectory;
        return true;
    }

    bool CpTrajectoryHistory::getLastTrajectory(moveit_msgs::RobotTrajectory &trajectory)
    {
        return getLastTrajectory(-1, trajectory);
    }

    void CpTrajectoryHistory::pushTrajectory(std::string name, const moveit_msgs::RobotTrajectory &trajectory, moveit_msgs::MoveItErrorCodes result)
    {
        TrajectoryHistoryEntry entry;
        this->trajectoryHistory_.push_back(entry);

        auto &last = this->trajectoryHistory_.back();
        last.trajectory = trajectory;
        last.result = result;
        last.name = name;
    }

} // namespace cl_move_group_interface