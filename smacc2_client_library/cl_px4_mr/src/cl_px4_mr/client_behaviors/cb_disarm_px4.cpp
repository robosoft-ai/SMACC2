#include <cl_px4_mr/client_behaviors/cb_disarm_px4.hpp>
#include <cl_px4_mr/components/cp_vehicle_command.hpp>
#include <cl_px4_mr/components/cp_vehicle_status.hpp>

namespace cl_px4_mr
{

CbDisarmPX4::CbDisarmPX4() {}

void CbDisarmPX4::onEntry()
{
  this->requiresComponent(vehicleCommand_);
  this->requiresComponent(vehicleStatus_);

  this->getStateMachine()->createSignalConnection(
    vehicleStatus_->onDisarmed_, &CbDisarmPX4::onDisarmedCallback, this);

  RCLCPP_INFO(getLogger(), "CbDisarmPX4: sending disarm command (attempt %d/%d)", retryCount_ + 1, MAX_RETRIES);
  vehicleCommand_->disarm();
}

void CbDisarmPX4::onExit() {}

void CbDisarmPX4::onDisarmedCallback()
{
  RCLCPP_INFO(getLogger(), "CbDisarmPX4: vehicle DISARMED - posting success");
  this->postSuccessEvent();
}

}  // namespace cl_px4_mr
