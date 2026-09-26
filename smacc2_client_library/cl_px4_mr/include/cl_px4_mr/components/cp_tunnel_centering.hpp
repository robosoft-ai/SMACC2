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
#include <chrono>
#include <mutex>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <smacc2/client_core_components/cp_topic_subscriber.hpp>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

struct TunnelCenteringParams
{
  // returns closer than this are the vehicle's own arms and rotors, not the cave;
  // without this cut the floor and ceiling both read 0 m and the vertical trim
  // is a constant +maxVerticalM climb (sm_cl_px4_mr_test_5 runs 16-21)
  float minRangeM = 0.6f;
  // slab ahead of the vehicle (level frame, metres) the walls are measured in
  float lookAheadMinM = 1.5f;
  float lookAheadMaxM = 6.0f;
  // half-height of the band, about the sensor, used for the wall measurement
  float wallBandHalfHeightM = 0.6f;
  // half-width of the column, about the heading, used for floor/ceiling; the
  // column runs from floorLookBackM BEHIND the sensor (what is under and beside
  // the vehicle counts, e.g. a ledge it hovers over) to lookAheadMaxM. Keep it
  // near the airframe's half-width: in a 5 m passage a wider column reaches the
  // walls, whose lower slopes then read as a floor. Points inside the wall band
  // (|z| <= wallBandHalfHeightM) are never floor or ceiling.
  float columnHalfWidthM = 1.0f;
  float floorLookBackM = 1.5f;
  float lateralSearchM = 8.0f;   // walls farther than this are "not seen"
  float verticalSearchM = 8.0f;  // floor/ceiling farther than this are "not seen"
  // desired height above the floor and minimum gap below the ceiling
  float floorClearanceM = 1.75f;
  float ceilingClearanceM = 1.0f;
  // The vertical correction never takes the vehicle closer to the measured floor
  // than this, whatever the overhead reading says. Rising terrain ahead (a rock
  // pile reaching sensor height) reads as a low "ceiling"; diving under it means
  // flying into the floor - that case is an obstacle for the guard, not a height
  // to trim to.
  float minFloorClearanceM = 1.0f;
  // corrections are clamped ...
  float maxLateralM = 2.0f;
  float maxVerticalM = 1.0f;
  // ... and the lateral one fades to zero when the nearer wall is farther than this
  float wallFadeM = 6.0f;
  int minWallPoints = 15;  // per side, else that side is "not seen"
  int minFloorPoints = 15;
  float smoothing = 0.6f;  // EMA weight of the previous value (0 = none)
  bool levelFrame = true;  // rotate the cloud by roll/pitch (needs /fmu/out/vehicle_attitude)
};

// Keeps the vehicle in the middle of whatever passage it is flying through.
// Each cloud (sensor frame, rotated level) is reduced to the nearest left wall,
// right wall, floor and ceiling in a slab ahead; from those it derives a
// lateral offset (toward the midpoint between the walls) and a vertical one
// (floorClearanceM above the floor, but ceilingClearanceM below the ceiling),
// clamped and smoothed. A path follower with useTunnelCentering set adds them
// to every carrot it commands (see CbPx4PathFollowerBase). No map, no memory:
// a purely reactive local planner for corridors, tunnels and caves.
//
// Requires a CpTopicSubscriber<sensor_msgs::msg::PointCloud2> on the same
// client; the sensor must be mounted level with the body (x forward, z up).
// onCloud runs on the ROS executor; consumers read atomics.
class CpTunnelCentering : public smacc2::ISmaccComponent
{
public:
  explicit CpTunnelCentering(TunnelCenteringParams params = {});
  virtual ~CpTunnelCentering();

  void onInitialize() override;

  // offsets in the level heading frame: lateral > 0 = move LEFT, vertical > 0 = climb
  float lateralOffsetM() const { return lateral_.load(); }
  float verticalOffsetM() const { return vertical_.load(); }
  // true while clouds keep arriving (fresher than 1 s)
  bool valid() const;

  // last measurement (metres from the sensor, +inf when not seen)
  float leftWallM() const { return left_.load(); }
  float rightWallM() const { return right_.load(); }
  float floorM() const { return floor_.load(); }
  float ceilingM() const { return ceiling_.load(); }

  const TunnelCenteringParams & params() const { return params_; }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2 & msg);
  void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);

  TunnelCenteringParams params_;
  smacc2::client_core_components::CpTopicSubscriber<sensor_msgs::msg::PointCloud2> * subscriber_ =
    nullptr;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitudeSub_;
  mutable std::mutex attitudeMutex_;
  std::array<float, 4> attitudeQ_{{1.0f, 0.0f, 0.0f, 0.0f}};
  bool haveAttitude_ = false;

  std::atomic<float> lateral_{0.0f};
  std::atomic<float> vertical_{0.0f};
  std::atomic<float> left_{0.0f};
  std::atomic<float> right_{0.0f};
  std::atomic<float> floor_{0.0f};
  std::atomic<float> ceiling_{0.0f};
  std::atomic<int64_t> lastCloudNs_{0};
  std::chrono::steady_clock::time_point lastLog_;
};

}  // namespace cl_px4_mr
