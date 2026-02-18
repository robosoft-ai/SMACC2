#pragma once

#include <smacc2/smacc.hpp>

#include <cl_px4_mr/components/cp_vehicle_command.hpp>
#include <cl_px4_mr/components/cp_offboard_keep_alive.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>
#include <cl_px4_mr/components/cp_vehicle_status.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>
#include <cl_px4_mr/components/cp_vehicle_command_ack.hpp>
#include <cl_px4_mr/components/cp_goal_checker.hpp>

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
