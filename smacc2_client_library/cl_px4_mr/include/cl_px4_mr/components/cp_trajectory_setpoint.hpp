#pragma once

#include <cmath>
#include <mutex>
#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

namespace cl_px4_mr
{

class CpVehicleLocalPosition;

class CpTrajectorySetpoint : public smacc2::ISmaccComponent
{
public:
  CpTrajectorySetpoint();
  virtual ~CpTrajectorySetpoint();

  void onInitialize() override;

  void setPositionNED(float x, float y, float z, float yaw = std::numeric_limits<float>::quiet_NaN());
  void hold();
  void republishLast();

  px4_msgs::msg::TrajectorySetpoint getLastSetpoint() const;

private:
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr publisher_;
  CpVehicleLocalPosition * localPosition_ = nullptr;
  px4_msgs::msg::TrajectorySetpoint lastSetpoint_;
  mutable std::mutex mutex_;
  bool hasPublished_ = false;
};

}  // namespace cl_px4_mr
