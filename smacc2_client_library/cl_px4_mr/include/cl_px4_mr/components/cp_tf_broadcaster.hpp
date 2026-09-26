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

#include <chrono>
#include <memory>
#include <string>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

namespace cl_px4_mr
{

class CpVehicleLocalPosition;

struct TfBroadcasterParams
{
  std::string mapFrame = "map";
  std::string baseFrame = "base_link";
  // same translation as baseFrame, identity rotation: an RViz orbit camera
  // targeting it follows the vehicle without spinning with its yaw
  std::string followFrame = "base_link_follow";
  std::string sensorFrame = "lidar_link";
  bool publishSensorFrame = true;
  // sensor mount in the body FLU frame (metres / radians)
  double sensorXyz[3] = {0.12, 0.0, 0.315};
  double sensorRpy[3] = {0.0, 0.0, 0.0};
  double maxRateHz = 50.0;  // vehicle_attitude arrives at 100-250 Hz
  // flown trail as a nav_msgs/Path in mapFrame (for RViz); a pose is appended
  // every trailMinStepM of travel, the path republished at <= trailRateHz
  bool publishTrail = true;
  std::string trailTopic = "/px4/trail";
  double trailMinStepM = 0.25;
  double trailRateHz = 2.0;
  size_t trailMaxPoses = 20000;
};

// Publishes the PX4 state estimate as a ROS TF tree: map -> base_link from
// /fmu/out/vehicle_attitude (rotation) and CpVehicleLocalPosition (translation),
// converted from PX4's NED / FRD conventions to ROS ENU / FLU, plus a
// translation-only map -> base_link_follow and a static base_link -> sensor
// frame. Optionally also publishes the flown trail as a nav_msgs/Path. `map` is the FMU's local NED origin (ref_lat / ref_lon), expressed
// ENU: x east, y north, z up. EKF origin resets are not compensated.
//
// Opt-in: not created by ClPx4Mr; an orthogonal that wants TF adds it with
//   createClient<ClPx4Mr>()->createComponent<CpTfBroadcaster>(params);
// Threading: onAttitude runs on the ROS executor and touches no state machine
// APIs.
class CpTfBroadcaster : public smacc2::ISmaccComponent
{
public:
  explicit CpTfBroadcaster(TfBroadcasterParams params = {});
  virtual ~CpTfBroadcaster();

  void onInitialize() override;

  const TfBroadcasterParams & params() const { return params_; }

private:
  void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);

  TfBroadcasterParams params_;
  CpVehicleLocalPosition * localPosition_ = nullptr;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitudeSub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticBroadcaster_;
  std::chrono::steady_clock::time_point lastSent_;
  bool firstSent_ = false;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trailPub_;
  nav_msgs::msg::Path trail_;
  std::chrono::steady_clock::time_point lastTrailSent_;
};

}  // namespace cl_px4_mr
