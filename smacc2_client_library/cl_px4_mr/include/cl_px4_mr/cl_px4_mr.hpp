// Copyright 2025 Robosoft Inc.
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

#pragma once

#include <smacc2/smacc.hpp>

#include <cl_px4_mr/components/cp_goal_checker.hpp>
#include <cl_px4_mr/components/cp_offboard_keep_alive.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_command.hpp>
#include <cl_px4_mr/components/cp_vehicle_command_ack.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>
#include <cl_px4_mr/components/cp_vehicle_status.hpp>

namespace cl_px4_mr
{

class ClPx4Mr : public smacc2::ISmaccClient
{
public:
  ClPx4Mr();
  virtual ~ClPx4Mr();

  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    this->createComponent<CpVehicleCommand, TOrthogonal, TClient>();
    this->createComponent<CpTrajectorySetpoint, TOrthogonal, TClient>();
    this->createComponent<CpVehicleLocalPosition, TOrthogonal, TClient>();
    this->createComponent<CpOffboardKeepAlive, TOrthogonal, TClient>();
    this->createComponent<CpVehicleStatus, TOrthogonal, TClient>();
    this->createComponent<CpVehicleCommandAck, TOrthogonal, TClient>();
    this->createComponent<CpGoalChecker, TOrthogonal, TClient>();
  }
};

}  // namespace cl_px4_mr
