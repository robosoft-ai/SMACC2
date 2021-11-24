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
// executes a circular trajectory of the tipLink around one axis defined
// by the z-axis of the plaenePivotPose
class CbCircularPivotMotion : public CbMoveEndEffectorTrajectory
{
public:
  std::optional<double> angularSpeed_rad_s_;
  std::optional<double> linearSpeed_m_s_;

  // if not specified it would be used the current robot pose
  std::optional<geometry_msgs::msg::Pose> relativeInitialPose_;

  CbCircularPivotMotion(std::optional<std::string> tipLink = std::nullopt);

  CbCircularPivotMotion(
    const geometry_msgs::msg::PoseStamped & planePivotPose, double deltaRadians,
    std::optional<std::string> tipLink = std::nullopt);

  CbCircularPivotMotion(
    const geometry_msgs::msg::PoseStamped & planePivotPose,
    const geometry_msgs::msg::Pose & relativeInitialPose, double deltaRadians,
    std::optional<std::string> tipLink = std::nullopt);

  virtual void generateTrajectory() override;

  virtual void createMarkers() override;

protected:
  // the rotation axis (z-axis) and the center of rotation in the space
  geometry_msgs::msg::PoseStamped planePivotPose_;

  double deltaRadians_;

private:
  void computeCurrentEndEffectorPoseRelativeToPivot();
};

}  // namespace cl_move_group_interface
