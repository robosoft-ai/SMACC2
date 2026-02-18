#pragma once

#include <atomic>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpVehicleCommand;
class CpVehicleStatus;

class CbArmPX4 : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbArmPX4();

  void onEntry() override;
  void onExit() override;

private:
  void onArmedCallback();

  CpVehicleCommand * vehicleCommand_ = nullptr;
  CpVehicleStatus * vehicleStatus_ = nullptr;
  std::atomic<bool> armed_{false};
  static constexpr int MAX_RETRIES = 5;
  static constexpr int RETRY_INTERVAL_SEC = 5;
};

}  // namespace cl_px4_mr
