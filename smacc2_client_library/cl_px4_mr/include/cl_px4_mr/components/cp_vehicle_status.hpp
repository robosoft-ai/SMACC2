#pragma once

#include <mutex>
#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace cl_px4_mr
{

class CpVehicleStatus : public smacc2::ISmaccComponent
{
public:
  CpVehicleStatus();
  virtual ~CpVehicleStatus();

  void onInitialize() override;

  bool isArmed() const;
  bool isLanded() const;
  uint8_t getNavState() const;
  uint8_t getArmingState() const;

  smacc2::SmaccSignal<void()> onArmed_;
  smacc2::SmaccSignal<void()> onDisarmed_;
  smacc2::SmaccSignal<void()> onModeChanged_;
  smacc2::SmaccSignal<void()> onLanded_;

private:
  void onStatusMessage(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr subscriber_;
  uint8_t armingState_ = 0;
  uint8_t navState_ = 0;
  uint8_t prevArmingState_ = 0;
  uint8_t prevNavState_ = 0;
  mutable std::mutex mutex_;
};

}  // namespace cl_px4_mr
