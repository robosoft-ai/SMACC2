// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

<<<<<<< HEAD:smacc2_client_library/moveit2z_client/src/moveit2z_client/client_behaviors/cb_execute_last_trajectory.cpp
#include <moveit2z_client/client_behaviors/cb_execute_last_trajectory.hpp>
#include <moveit2z_client/components/cp_trajectory_history.hpp>
=======
#include <moveit2z_client/client_behaviors/cb_execute_last_trajectory.hpp>
#include <moveit2z_client/components/cp_trajectory_history.hpp>
>>>>>>> 056c654b26293282493ab9a4aaec5399f25f061f:smacc2_client_library/moveit2z_client/src/moveit2z_client/client_behaviors/cb_execute_last_trajectory.cpp

namespace cl_moveit2z
{
  CbExecuteLastTrajectory::CbExecuteLastTrajectory() {}

  CbExecuteLastTrajectory::~CbExecuteLastTrajectory() {}

  void CbExecuteLastTrajectory::generateTrajectory() {}

  void CbExecuteLastTrajectory::onEntry()
  {
    this->requiresClient(movegroupClient_);

    CpTrajectoryHistory * trajectoryHistory;
    this->requiresComponent(trajectoryHistory);

    // this->generateTrajectory();
    // endEffectorTrajectory_ =

    // if (this->endEffectorTrajectory_.size() == 0)
    // {
    //     RCLCPP_WARN_STREAM(getLogger(), "[" << smacc2::demangleSymbol(typeid(*this).name()) << "] No points in the trajectory. Skipping behavior.");
    //     return;
    // }

    //this->createMarkers();
    //markersInitialized_ = true;

    moveit_msgs::msg::RobotTrajectory trajectory;

    if (trajectoryHistory->getLastTrajectory(trajectory))
    {
      this->executeJointSpaceTrajectory(trajectory);
    }
  }

}  // namespace cl_moveit2z
