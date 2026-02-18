#pragma once

#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpVehicleCommand;
class CpVehicleStatus;

class CbDisarmPX4 : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbDisarmPX4();

  void onEntry() override;
  void onExit() override;

private:
  void onDisarmedCallback();

  CpVehicleCommand * vehicleCommand_ = nullptr;
  CpVehicleStatus * vehicleStatus_ = nullptr;
  int retryCount_ = 0;
  static constexpr int MAX_RETRIES = 3;
};

}  // namespace cl_px4_mr
