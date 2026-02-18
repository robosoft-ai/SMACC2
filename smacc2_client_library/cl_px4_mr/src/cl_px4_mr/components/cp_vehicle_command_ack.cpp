#include <cl_px4_mr/components/cp_vehicle_command_ack.hpp>

namespace cl_px4_mr
{

CpVehicleCommandAck::CpVehicleCommandAck() {}

CpVehicleCommandAck::~CpVehicleCommandAck() {}

void CpVehicleCommandAck::onInitialize()
{
  auto node = this->getNode();
  subscriber_ = node->create_subscription<px4_msgs::msg::VehicleCommandAck>(
    "/fmu/out/vehicle_command_ack",
    rclcpp::SensorDataQoS(),
    std::bind(&CpVehicleCommandAck::onAckMessage, this, std::placeholders::_1));
  RCLCPP_INFO(getLogger(), "CpVehicleCommandAck: subscribed to /fmu/out/vehicle_command_ack");
}

void CpVehicleCommandAck::onAckMessage(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
{
  lastCommand_ = msg->command;
  lastResult_ = msg->result;
  RCLCPP_INFO(getLogger(), "CpVehicleCommandAck: command=%u result=%u", msg->command, msg->result);
  onAckReceived_();
}

uint32_t CpVehicleCommandAck::getLastAckCommand() const
{
  return lastCommand_;
}

uint8_t CpVehicleCommandAck::getLastAckResult() const
{
  return lastResult_;
}

}  // namespace cl_px4_mr
