#pragma once

#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpVehicleCommand;
class CpOffboardKeepAlive;
class CpTrajectorySetpoint;
class CpGoalChecker;
class CpVehicleLocalPosition;

class CbTakeOff : public smacc2::SmaccAsyncClientBehavior
{
public:
  explicit CbTakeOff(float targetAltitude = 5.0f);

  void onEntry() override;
  void onExit() override;

private:
  void onGoalReachedCallback();

  float targetAltitude_;
  CpVehicleCommand * vehicleCommand_ = nullptr;
  CpOffboardKeepAlive * offboardKeepAlive_ = nullptr;
  CpTrajectorySetpoint * trajectorySetpoint_ = nullptr;
  CpGoalChecker * goalChecker_ = nullptr;
  CpVehicleLocalPosition * localPosition_ = nullptr;
};

}  // namespace cl_px4_mr
