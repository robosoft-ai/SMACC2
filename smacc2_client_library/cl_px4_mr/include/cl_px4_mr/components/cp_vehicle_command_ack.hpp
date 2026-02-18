#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>

namespace cl_px4_mr
{

class CpVehicleCommandAck : public smacc2::ISmaccComponent
{
public:
  CpVehicleCommandAck();
  virtual ~CpVehicleCommandAck();

  void onInitialize() override;

  uint32_t getLastAckCommand() const;
  uint8_t getLastAckResult() const;

  smacc2::SmaccSignal<void()> onAckReceived_;

private:
  void onAckMessage(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr subscriber_;
  uint32_t lastCommand_ = 0;
  uint8_t lastResult_ = 0;
};

}  // namespace cl_px4_mr
