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
 *****************************************************************************************************************/
#pragma once

#include <tf2/transform_datatypes.h>
#include "cb_move_end_effector_trajectory.hpp"

namespace cl_move_group_interface
{
class CbCircularPouringMotion : public CbMoveEndEffectorTrajectory
{
public:
  std::optional<double> angularSpeed_rad_s_;

  std::optional<double> linearSpeed_m_s_;

  CbCircularPouringMotion(
    const geometry_msgs::msg::Point & pivotPoint, double deltaHeight, std::string tipLink,
    std::string globalFrame);

  virtual void generateTrajectory() override;

  virtual void createMarkers() override;

  geometry_msgs::msg::Vector3 directionVector_;

  // relative position of the "lid" of the bottle in the end effector reference frame.
  // that point is the one that must do the linear motion
  geometry_msgs::msg::Pose pointerRelativePose_;

protected:
  geometry_msgs::msg::Point relativePivotPoint_;

  // distance in meters from the initial pose to the bottom/top direction in z axe
  double deltaHeight_;

  std::vector<geometry_msgs::msg::PoseStamped> pointerTrajectory_;

private:
  void computeCurrentEndEffectorPoseRelativeToPivot();

  std::string globalFrame_;
};

}  // namespace cl_move_group_interface
