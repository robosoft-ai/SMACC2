// Copyright 2026 RobosoftAI Inc.
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

/*****************************************************************************************************************
 *
 * 	 Authors: Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <array>
#include <atomic>
#include <cmath>
#include <mutex>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <smacc2/client_core_components/cp_topic_subscriber.hpp>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

// state machine events, posted by CbObstacleGuard on the guard's signals
template <typename TSource, typename TOrthogonal>
struct EvObstacleTooClose : sc::event<EvObstacleTooClose<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvObstacleCleared : sc::event<EvObstacleCleared<TSource, TOrthogonal>>
{
};

struct ForwardObstacleGuardParams
{
  float halfAngleRad = 0.349f;  // 20 deg cone about the sensor +x axis
  float triggerRangeM = 3.0f;   // too close below this ...
  float clearRangeM = 4.0f;     // ... cleared again above this (hysteresis)
  float minRangeM = 0.6f;       // ignore the airframe's own returns
  // Ignore returns more than this far below the sensor (metres, sensor frame):
  // the floor of a passage flown at low altitude must not count as an obstacle
  // ahead; anything reaching up to within this of the vehicle still does. 0 disables.
  float floorCutoffM = 0.8f;
  int minHits = 10;       // cone points below triggerRangeM per cloud to count
  int triggerClouds = 2;  // consecutive triggering clouds before "too close"
  int clearClouds = 10;   // consecutive clear clouds before "cleared"
  // Clouds captured while the vehicle yaws faster than this (rad/s, derived from
  // successive /fmu/out/vehicle_attitude headings) are ignored: the cone is
  // sweeping the walls, not looking along the direction of travel (e.g. the
  // 180 deg turn at the start of a retrace). 0 disables the gate.
  float maxYawRateRadS = 0.35f;
  // Evaluate the cone in a level frame: each cloud is rotated by the vehicle's
  // roll and pitch (from /fmu/out/vehicle_attitude) so the cone axis is the
  // true heading and floorCutoffM is true height below the sensor. Without it
  // a nose-down pitch tilts the floor up into the cone.
  bool levelFrame = true;
};

// Watches a PointCloud2 (sensor frame) for returns inside a forward cone and
// raises / clears an obstacle flag with hysteresis. "Forward" is the sensor's
// +x axis: with the lidar mounted level and every leg flown in
// YawMode::TANGENT, body x is the direction of travel while cruising. The
// cone is not the velocity direction during vertical legs, holds, or the
// 180-degree turn at the start of a retrace - keep the trigger range below
// half the narrowest passage width so those sweeps do not trip it.
//
// Requires a CpTopicSubscriber<sensor_msgs::msg::PointCloud2> on the same
// client (ClGenericSensor<PointCloud2> creates one). Subscribes to PX4's
// /fmu/out/vehicle_attitude itself (level frame + yaw-rate gate). The sensor must be mounted level with the body (x
// forward, z up). onCloud runs on the ROS executor; it only emits SmaccSignals.
class CpForwardObstacleGuard : public smacc2::ISmaccComponent
{
public:
  explicit CpForwardObstacleGuard(ForwardObstacleGuardParams params = {});
  virtual ~CpForwardObstacleGuard();

  void onInitialize() override;

  template <typename T>
  smacc2::SmaccSignalConnection onObstacleTooClose(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onObstacleTooClose_, callback, object);
  }

  template <typename T>
  smacc2::SmaccSignalConnection onObstacleCleared(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onObstacleCleared_, callback, object);
  }

  // forget the current flag and debounce state: a behavior arming itself calls
  // this so a stale flag (e.g. the ground seen while sitting on the pad) does
  // not fire the moment it connects; a real obstacle re-triggers within
  // triggerClouds clouds
  void reset();

  bool isTooClose() const { return tooClose_.load(); }
  float lastYawRate() const { return yawRate_.load(); }
  float lastMinRange() const { return lastMinRange_.load(); }
  int lastHits() const { return lastHits_.load(); }
  const ForwardObstacleGuardParams & params() const { return params_; }

  smacc2::SmaccSignal<void()> onObstacleTooClose_;
  smacc2::SmaccSignal<void()> onObstacleCleared_;

private:
  void onCloud(const sensor_msgs::msg::PointCloud2 & msg);
  void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
  // roll/pitch-only rotation (sensor -> level frame) from the latest attitude; false if none yet
  bool levelRotation(std::array<float, 9> & r) const;

  ForwardObstacleGuardParams params_;
  smacc2::client_core_components::CpTopicSubscriber<sensor_msgs::msg::PointCloud2> * subscriber_ =
    nullptr;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitudeSub_;
  double lastYaw_ = 0.0;
  uint64_t lastYawStampUs_ = 0;
  mutable std::mutex attitudeMutex_;
  std::array<float, 4> attitudeQ_{{1.0f, 0.0f, 0.0f, 0.0f}};  // PX4 order w, x, y, z
  bool haveAttitude_ = false;
  std::atomic<float> yawRate_{0.0f};
  int gatedClouds_ = 0;
  float cosHalfAngle_ = 1.0f;
  int triggerStreak_ = 0;
  int clearStreak_ = 0;
  std::atomic<bool> tooClose_{false};
  std::atomic<float> lastMinRange_{std::numeric_limits<float>::infinity()};
  std::atomic<int> lastHits_{0};
};

}  // namespace cl_px4_mr
