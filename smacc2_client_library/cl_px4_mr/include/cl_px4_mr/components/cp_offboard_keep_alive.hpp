#pragma once

#include <smacc2/smacc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

namespace cl_px4_mr
{

class CpTrajectorySetpoint;

class CpOffboardKeepAlive : public smacc2::ISmaccComponent, public smacc2::ISmaccUpdatable
{
public:
  CpOffboardKeepAlive();
  virtual ~CpOffboardKeepAlive();

  void onInitialize() override;
  void update() override;

  void enable();
  void disable();
  bool isEnabled() const;

private:
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr publisher_;
  CpTrajectorySetpoint * trajectorySetpoint_ = nullptr;
  bool enabled_ = false;
};

}  // namespace cl_px4_mr
