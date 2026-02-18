#pragma once

#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpVehicleCommand;
class CpVehicleStatus;
class CpOffboardKeepAlive;

class CbLand : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbLand();

  void onEntry() override;
  void onExit() override;

private:
  void onLandedCallback();

  CpVehicleCommand * vehicleCommand_ = nullptr;
  CpVehicleStatus * vehicleStatus_ = nullptr;
  CpOffboardKeepAlive * offboardKeepAlive_ = nullptr;
};

}  // namespace cl_px4_mr
