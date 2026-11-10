// Copyright 2025 Robosoft Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cl_px4_mr/client_behaviors/cb_connect_micro_ros_agent.hpp>
#include <cl_px4_mr/components/cp_failsafe_flags.hpp>
#include <cl_px4_mr/components/cp_micro_ros_agent.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

#include <chrono>

namespace cl_px4_mr
{

CbConnectMicroRosAgent::CbConnectMicroRosAgent(double timeoutSec)
: timeoutSec_(timeoutSec), rate_(5)
{
}

void CbConnectMicroRosAgent::onEntry()
{
  if (!microRosAgent_->isLaunched())
  {
    RCLCPP_INFO(getLogger(), "CbConnectMicroRosAgent: launching micro_ros_agent...");
    microRosAgent_->launch();
  }

  std::string targetNodeName = microRosAgent_->getNodeName();
  RCLCPP_INFO(
    getLogger(), "CbConnectMicroRosAgent: waiting for node '%s' (timeout %.1fs)",
    targetNodeName.c_str(), timeoutSec_);

  auto startTime = std::chrono::steady_clock::now();
  bool found = false;

  while (!this->isShutdownRequested() && !found)
  {
    // Check timeout
    auto elapsed = std::chrono::steady_clock::now() - startTime;
    double elapsedSec = std::chrono::duration<double>(elapsed).count();
    if (elapsedSec > timeoutSec_)
    {
      RCLCPP_ERROR(
        getLogger(), "CbConnectMicroRosAgent: timeout (%.1fs) waiting for '%s'", timeoutSec_,
        targetNodeName.c_str());
      this->postPx4Failure();
      return;
    }

    // Poll for node
    std::stringstream ss;
    auto nodeNames = getNode()->get_node_names();

    for (const auto & n : nodeNames)
    {
      ss << " - " << n << std::endl;

      if (n == targetNodeName)
      {
        found = true;
      }
    }

    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] listing nodes (" << nodeNames.size() << ")" << std::endl
                       << ss.str());

    rate_.sleep();
  }

  if (!found)
  {
    RCLCPP_WARN(getLogger(), "CbConnectMicroRosAgent: shutdown requested before node found");
    this->postPx4Failure();
    return;
  }

  RCLCPP_INFO(
    getLogger(), "CbConnectMicroRosAgent: node '%s' detected - starting failsafe health check",
    targetNodeName.c_str());

  // Phase 2: Wait for PX4 failsafe health flags to clear, read from the state-machine-lifetime
  // CpFailsafeFlags component, so no subscription callback can run into this behavior at dispose.
  if (failsafeFlags_ == nullptr)
  {
    RCLCPP_ERROR(getLogger(), "CbConnectMicroRosAgent: no CpFailsafeFlags component on the client");
    this->postPx4Failure();
    return;
  }

  // the failsafe flags say an estimate exists; the local position and velocity
  // also have to be valid. NOT heading_good_for_control: with a magnetometer
  // heading PX4 only sets it after the in-flight yaw alignment (moving with GPS
  // course), so on the ground it is false by design; it is logged for diagnosis.
  auto controlReady = [this]()
  {
    return localPosition_ == nullptr ||
           (localPosition_->isValid() && localPosition_->isVelocityValid());
  };
  auto flag = [this](bool (CpVehicleLocalPosition::*f)() const)
  { return localPosition_ ? static_cast<int>((localPosition_->*f)()) : -1; };
  rclcpp::Rate healthRate(2.0);
  while (!this->isShutdownRequested() && !(failsafeFlags_->isHealthOk() && controlReady()))
  {
    auto elapsed = std::chrono::steady_clock::now() - startTime;
    double elapsedSec = std::chrono::duration<double>(elapsed).count();
    if (elapsedSec > timeoutSec_)
    {
      RCLCPP_ERROR(
        getLogger(),
        "CbConnectMicroRosAgent: timeout (%.1fs) waiting for health check. "
        "attitude_invalid=%d, local_altitude_invalid=%d, local_position_invalid=%d, "
        "heading_good_for_control=%d, velocity_valid=%d",
        timeoutSec_, failsafeFlags_->isAttitudeInvalid(), failsafeFlags_->isLocalAltitudeInvalid(),
        failsafeFlags_->isLocalPositionInvalid(),
        flag(&CpVehicleLocalPosition::isHeadingGoodForControl),
        flag(&CpVehicleLocalPosition::isVelocityValid));
      this->postPx4Failure();
      return;
    }
    RCLCPP_INFO(
      getLogger(),
      "CbConnectMicroRosAgent: health check: attitude_invalid=%d, "
      "local_altitude_invalid=%d, local_position_invalid=%d, heading_good_for_control=%d, "
      "velocity_valid=%d",
      failsafeFlags_->isAttitudeInvalid(), failsafeFlags_->isLocalAltitudeInvalid(),
      failsafeFlags_->isLocalPositionInvalid(),
      flag(&CpVehicleLocalPosition::isHeadingGoodForControl),
      flag(&CpVehicleLocalPosition::isVelocityValid));
    healthRate.sleep();
  }
  if (failsafeFlags_->isHealthOk() && controlReady())
  {
    RCLCPP_INFO(getLogger(), "CbConnectMicroRosAgent: health check passed - posting success");
    this->postPx4Success();
  }
  else
  {
    RCLCPP_WARN(
      getLogger(), "CbConnectMicroRosAgent: shutdown requested before health check passed");
    this->postPx4Failure();
  }
}

}  // namespace cl_px4_mr
