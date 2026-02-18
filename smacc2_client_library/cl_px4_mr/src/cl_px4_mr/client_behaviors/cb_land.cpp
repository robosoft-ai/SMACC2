#include <cl_px4_mr/client_behaviors/cb_land.hpp>
#include <cl_px4_mr/components/cp_vehicle_command.hpp>
#include <cl_px4_mr/components/cp_vehicle_status.hpp>
#include <cl_px4_mr/components/cp_offboard_keep_alive.hpp>

namespace cl_px4_mr
{

CbLand::CbLand() {}

void CbLand::onEntry()
{
  this->requiresComponent(vehicleCommand_);
  this->requiresComponent(vehicleStatus_);
  this->requiresComponent(offboardKeepAlive_);

  // Connect to disarmed signal - PX4 auto-disarms after landing
  this->getStateMachine()->createSignalConnection(
    vehicleStatus_->onDisarmed_, &CbLand::onLandedCallback, this);

  // Disable offboard keepalive - land command uses its own mode
  offboardKeepAlive_->disable();

  RCLCPP_INFO(getLogger(), "CbLand: sending land command");
  vehicleCommand_->land();
}

void CbLand::onExit() {}

void CbLand::onLandedCallback()
{
  RCLCPP_INFO(getLogger(), "CbLand: vehicle landed and disarmed - posting success");
  this->postSuccessEvent();
}

}  // namespace cl_px4_mr
