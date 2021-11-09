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
#include <move_group_interface_client/cl_movegroup.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "cb_move_end_effector_trajectory.hpp"

namespace cl_move_group_interface
{
class CbMoveCartesianRelative2 : public CbMoveEndEffectorTrajectory
{
public:
  geometry_msgs::msg::Vector3 offset_;

  std::optional<double> linearSpeed_m_s_;

  CbMoveCartesianRelative2(std::string referenceFrame, std::string tipLink);

  CbMoveCartesianRelative2(
    std::string referenceFrame, std::string tipLink, geometry_msgs::msg::Vector3 offset);

  virtual ~CbMoveCartesianRelative2();

  void generateTrajectory() override;

private:
  std::string globalFrame_;
};
}  // namespace cl_move_group_interface
