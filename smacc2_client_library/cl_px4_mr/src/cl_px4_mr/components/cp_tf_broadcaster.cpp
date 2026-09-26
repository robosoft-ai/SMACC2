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

#include <cl_px4_mr/components/cp_tf_broadcaster.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

#include <vector>

namespace cl_px4_mr
{

namespace
{
// px4_ros_com frame_transforms constants (tf2 order x, y, z, w):
// NED <-> ENU is RotZ(+90 deg) * RotX(180 deg), its own inverse.
const tf2::Quaternion kNedEnuQ(0.70710678118654752, 0.70710678118654752, 0.0, 0.0);
// FRD <-> FLU is RotX(180 deg), its own inverse.
const tf2::Quaternion kFrdFluQ(1.0, 0.0, 0.0, 0.0);
}  // namespace

CpTfBroadcaster::CpTfBroadcaster(TfBroadcasterParams params) : params_(params) {}

CpTfBroadcaster::~CpTfBroadcaster() {}

void CpTfBroadcaster::onInitialize()
{
  this->requiresComponent(localPosition_);
  if (localPosition_ == nullptr)
  {
    RCLCPP_ERROR(
      getLogger(), "CpTfBroadcaster: CpVehicleLocalPosition not found - no TF will be published");
    return;
  }

  auto node = this->getNode();
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  staticBroadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  if (params_.publishSensorFrame)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node->now();
    t.header.frame_id = params_.baseFrame;
    t.child_frame_id = params_.sensorFrame;
    t.transform.translation.x = params_.sensorXyz[0];
    t.transform.translation.y = params_.sensorXyz[1];
    t.transform.translation.z = params_.sensorXyz[2];
    tf2::Quaternion q;
    q.setRPY(params_.sensorRpy[0], params_.sensorRpy[1], params_.sensorRpy[2]);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    staticBroadcaster_->sendTransform(t);
    RCLCPP_INFO(
      getLogger(), "CpTfBroadcaster: static %s -> %s at FLU (%.3f, %.3f, %.3f)",
      params_.baseFrame.c_str(), params_.sensorFrame.c_str(), params_.sensorXyz[0],
      params_.sensorXyz[1], params_.sensorXyz[2]);
  }

  if (params_.publishTrail)
  {
    trailPub_ = node->create_publisher<nav_msgs::msg::Path>(params_.trailTopic, rclcpp::QoS(1).transient_local());
    trail_.header.frame_id = params_.mapFrame;
    RCLCPP_INFO(
      getLogger(), "CpTfBroadcaster: publishing the flown trail on %s (every %.2f m, <= %.0f Hz)",
      params_.trailTopic.c_str(), params_.trailMinStepM, params_.trailRateHz);
  }

  attitudeSub_ = node->create_subscription<px4_msgs::msg::VehicleAttitude>(
    "/fmu/out/vehicle_attitude", rclcpp::SensorDataQoS(),
    std::bind(&CpTfBroadcaster::onAttitude, this, std::placeholders::_1));

  RCLCPP_INFO(
    getLogger(), "CpTfBroadcaster: broadcasting %s -> %s (and %s) from /fmu/out/vehicle_attitude at <= %.0f Hz",
    params_.mapFrame.c_str(), params_.baseFrame.c_str(), params_.followFrame.c_str(),
    params_.maxRateHz);
}

void CpTfBroadcaster::onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
{
  const auto now = std::chrono::steady_clock::now();
  if (firstSent_ && params_.maxRateHz > 0.0)
  {
    const double elapsed = std::chrono::duration<double>(now - lastSent_).count();
    if (elapsed < 1.0 / params_.maxRateHz)
    {
      return;
    }
  }
  if (!localPosition_->isValid())
  {
    return;
  }

  // PX4 quaternion is (w, x, y, z), rotation FRD body -> NED earth
  const tf2::Quaternion qNedFrd(msg->q[1], msg->q[2], msg->q[3], msg->q[0]);
  tf2::Quaternion qEnuFlu = kNedEnuQ * qNedFrd * kFrdFluQ;
  qEnuFlu.normalize();

  const auto stamp = this->getNode()->now();

  geometry_msgs::msg::TransformStamped base;
  base.header.stamp = stamp;
  base.header.frame_id = params_.mapFrame;
  base.child_frame_id = params_.baseFrame;
  base.transform.translation.x = localPosition_->getY();   // east
  base.transform.translation.y = localPosition_->getX();   // north
  base.transform.translation.z = -localPosition_->getZ();  // up
  base.transform.rotation.x = qEnuFlu.x();
  base.transform.rotation.y = qEnuFlu.y();
  base.transform.rotation.z = qEnuFlu.z();
  base.transform.rotation.w = qEnuFlu.w();

  geometry_msgs::msg::TransformStamped follow = base;
  follow.child_frame_id = params_.followFrame;
  follow.transform.rotation.x = 0.0;
  follow.transform.rotation.y = 0.0;
  follow.transform.rotation.z = 0.0;
  follow.transform.rotation.w = 1.0;

  broadcaster_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{base, follow});
  lastSent_ = now;

  if (trailPub_)
  {
    const auto & t = base.transform.translation;
    bool append = trail_.poses.empty();
    if (!append)
    {
      const auto & l = trail_.poses.back().pose.position;
      const double dx = t.x - l.x, dy = t.y - l.y, dz = t.z - l.z;
      append = std::sqrt(dx * dx + dy * dy + dz * dz) >= params_.trailMinStepM;
    }
    if (append)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = base.header;
      ps.pose.position.x = t.x;
      ps.pose.position.y = t.y;
      ps.pose.position.z = t.z;
      ps.pose.orientation = base.transform.rotation;
      trail_.poses.push_back(ps);
      if (trail_.poses.size() > params_.trailMaxPoses)
      {
        trail_.poses.erase(trail_.poses.begin());
      }
      const double sinceTrail = std::chrono::duration<double>(now - lastTrailSent_).count();
      if (params_.trailRateHz <= 0.0 || sinceTrail >= 1.0 / params_.trailRateHz)
      {
        trail_.header.stamp = stamp;
        trailPub_->publish(trail_);
        lastTrailSent_ = now;
      }
    }
  }
  if (!firstSent_)
  {
    firstSent_ = true;
    RCLCPP_INFO(
      getLogger(), "CpTfBroadcaster: first %s -> %s sent (ENU %.2f, %.2f, %.2f)",
      params_.mapFrame.c_str(), params_.baseFrame.c_str(), base.transform.translation.x,
      base.transform.translation.y, base.transform.translation.z);
  }
}

}  // namespace cl_px4_mr
