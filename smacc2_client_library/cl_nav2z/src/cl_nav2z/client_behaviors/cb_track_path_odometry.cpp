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

#pragma once

#include <cl_nav2z/client_behaviors/cb_track_path_odometry.hpp>
#include <cl_nav2z/components/odom_tracker/cp_odom_tracker.hpp>
#include <cl_nav2z/components/pose/cp_pose.hpp>

namespace cl_nav2z
{
using namespace smacc2;
CbTrackPathOdometry::CbTrackPathOdometry() {}

void CbTrackPathOdometry::onEntry()
{
  RCLCPP_INFO(this->getLogger(), "Pose tracker freeze reference frame");
  cl_nav2z::CpPose * poseComponent;
  requiresComponent(poseComponent, ComponentRequirement::HARD);
  poseComponent->freezeReferenceFrame();

  // poseComponent->setReferenceFrame("odom");

  RCLCPP_INFO(this->getLogger(), "Odom tracker clear path");
  cl_nav2z::odom_tracker::CpOdomTracker * odomTracker;
  this->requiresComponent(odomTracker, ComponentRequirement::HARD);
  // odomTracker->setOdomFrame("odom");

  odomTracker->clearPath();
}

void CbTrackPathOdometry::onExit() {}
}  // namespace cl_nav2z
