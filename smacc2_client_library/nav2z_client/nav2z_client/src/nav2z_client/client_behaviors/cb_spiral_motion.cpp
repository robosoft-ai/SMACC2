// Copyright 2021 RobosoftAI Inc.
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
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <angles/angles.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2z_client/client_behaviors/cb_spiral_motion.hpp>
#include <optional>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_nav2z
{

CbSpiralMotion::CbSpiralMotion(std::optional<CbSpiralMotionOptions> options)
{
  if (options)
  {
    options_ = *options;
  }
  else
  {
    // we use default options
    options_ = CbSpiralMotionOptions();
  }
}

void CbSpiralMotion::onEntry()
{
  /*
 struct CbSpiralMotionOptions
{
  std::optional<float> linearVelocity = 0.0f;
  std::optional<float> maxLinearVelocity = 1.0f;
  std::optional<float> initialAngularVelocity = 1.0f;
  std::optional<rclcpp::Duration> spiralMotionDuration = rclcpp::Duration::from_seconds(120);
  std::optional <float> finalRadius=10.0f;//meters
};
   */

  auto linearVelocity = *(options_.linearVelocity);
  auto maxLinearVelocity = *(options_.maxLinearVelocity);
  auto initialAngularVelocity = *(options_.initialAngularVelocity);
  auto spiralMotionDuration = *(options_.spiralMotionDuration);
  auto finalRadius = *(options_.finalRadius);

  float rate = 20.0f;
  rclcpp::Rate r(rate);
  cmdVelPub_ = getNode()->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(1));

  rclcpp::Duration linearRamp = rclcpp::Duration::from_seconds(spiralMotionDuration.seconds());
  float linearAceleration = (maxLinearVelocity - linearVelocity) / linearRamp.seconds();
  float dt = 1.0f / rate;

  // we know final radious and the constant linear velocity
  float finalAngularVelocity = maxLinearVelocity / finalRadius;

  float angularAcceleration =
    (initialAngularVelocity - finalAngularVelocity) / spiralMotionDuration.seconds();

  geometry_msgs::msg::Twist cmd_vel;

  cmd_vel.linear.x = linearVelocity;
  cmd_vel.angular.z = initialAngularVelocity;
  auto start_time = getNode()->now();

  RCLCPP_INFO_STREAM(
    getLogger(), "[CbSpiralMotion]: initialAngularVelocity: "
                   << initialAngularVelocity << ", finalAngularVelocity: " << finalAngularVelocity
                   << ", angularAcceleration: " << angularAcceleration);
  RCLCPP_INFO_STREAM(
    getLogger(), "[CbSpiralMotion]: linearAceleration: "
                   << linearAceleration << ", maxLinearVelocity: " << maxLinearVelocity);

  bool end_condition = false;

  while (!end_condition)
  {
    auto current_time = getNode()->now() - start_time;

    cmd_vel.linear.x += linearAceleration * dt;
    if (cmd_vel.linear.x > maxLinearVelocity)
    {
      cmd_vel.linear.x = maxLinearVelocity;
    }

    float signVal = (cmd_vel.angular.z >= 0.0f) ? 1.0f : -1.0f;
    // cmd_vel.angular.z -= signVal * angularAcceleration * dt;

    float ellapsedTimeFactor = current_time.seconds() / spiralMotionDuration.seconds();
    cmd_vel.angular.z = initialAngularVelocity * (1.0f - ellapsedTimeFactor) +
                        finalAngularVelocity * ellapsedTimeFactor;

    RCLCPP_INFO(
      getLogger(), "[CbSpiralMotion] cmd_vel.linear.x = %f, cmd_vel.angular.z = %f",
      cmd_vel.linear.x, cmd_vel.angular.z);

    cmdVelPub_->publish(cmd_vel);
    r.sleep();

    auto now = getNode()->now();

    rclcpp::Duration ellapsed = now - start_time;
    RCLCPP_INFO_STREAM(
      getLogger(), "[CbSpiralMotion] ellapsed time: " << ellapsed.seconds() << ", total duration: "
                                                      << spiralMotionDuration.seconds());
    if (ellapsed > spiralMotionDuration)
    {
      RCLCPP_INFO_STREAM(getLogger(), "[CbSpiralMotion] spiralMotionDuration reached");
      end_condition = true;
    }
  }

  // asynchronous client behaviors usually post a success event when they are done
  // that is used in states to transition to the next state
  this->postSuccessEvent();
}

void CbSpiralMotion::onExit() {}

}  // namespace cl_nav2z
