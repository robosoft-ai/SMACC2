#pragma once

#include <mutex>
#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

namespace cl_px4_mr
{

class CpVehicleLocalPosition : public smacc2::ISmaccComponent
{
public:
  CpVehicleLocalPosition();
  virtual ~CpVehicleLocalPosition();

  void onInitialize() override;

  float getX() const;
  float getY() const;
  float getZ() const;
  float getHeading() const;
  bool isValid() const;

  smacc2::SmaccSignal<void()> onPositionReceived_;

private:
  void onPositionMessage(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscriber_;
  float x_ = 0.0f;
  float y_ = 0.0f;
  float z_ = 0.0f;
  float heading_ = 0.0f;
  bool valid_ = false;
  mutable std::mutex mutex_;
};

}  // namespace cl_px4_mr
