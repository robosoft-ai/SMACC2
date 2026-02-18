#include <cl_px4_mr/components/cp_offboard_keep_alive.hpp>
#include <cl_px4_mr/components/cp_trajectory_setpoint.hpp>

namespace cl_px4_mr
{

CpOffboardKeepAlive::CpOffboardKeepAlive() {}

CpOffboardKeepAlive::~CpOffboardKeepAlive() {}

void CpOffboardKeepAlive::onInitialize()
{
  auto node = this->getNode();
  publisher_ = node->create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", rclcpp::QoS(1).best_effort());

  this->requiresComponent(trajectorySetpoint_);

  RCLCPP_INFO(getLogger(), "CpOffboardKeepAlive: initialized (disabled)");
}

void CpOffboardKeepAlive::update()
{
  if (!enabled_) return;

  auto node = this->getNode();

  px4_msgs::msg::OffboardControlMode msg;
  msg.timestamp = node->get_clock()->now().nanoseconds() / 1000;
  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;
  publisher_->publish(msg);

  // Republish the latest trajectory setpoint to keep PX4 happy
  if (trajectorySetpoint_)
  {
    trajectorySetpoint_->republishLast();
  }
}

void CpOffboardKeepAlive::enable()
{
  enabled_ = true;
  RCLCPP_INFO(getLogger(), "CpOffboardKeepAlive: ENABLED");
}

void CpOffboardKeepAlive::disable()
{
  enabled_ = false;
  RCLCPP_INFO(getLogger(), "CpOffboardKeepAlive: DISABLED");
}

bool CpOffboardKeepAlive::isEnabled() const
{
  return enabled_;
}

}  // namespace cl_px4_mr
