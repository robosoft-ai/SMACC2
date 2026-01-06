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

#pragma once

#include <cl_nav2z/components/pose/cp_pose.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_nav2z
{

// client behaviors typically accept parameters to be customized in the state it is being used
// we follow this pattern, with a basic structure with optional fields, that make thee initialization of the behavior more readable in the
// state initialization. It also avoid an explosion of constructors for each variation of the parameters.
struct CbSpiralMotionOptions
{
  std::optional<float> linearVelocity = 0.0f;
  std::optional<float> maxLinearVelocity = 0.5f;
  std::optional<float> initialAngularVelocity = 1.5f;
  std::optional<rclcpp::Duration> spiralMotionDuration = rclcpp::Duration::from_seconds(40);
  std::optional<float> finalRadius = 20.0f;  //meters
};

/* a basic client behavior has a simple onEntry and onExit functions. When we enter into a stae we call onEntry.
This client behavior is asynchronous so that onEntry returns immediately, it opens a thread to run the behavior*/
struct CbSpiralMotion : public smacc2::SmaccAsyncClientBehavior
{
public:
  //
  // constructor, we accept an empty usag of the behavior, but also any other combination of parameters
  CbSpiralMotion(std::optional<CbSpiralMotionOptions> options = std::nullopt);

  void onEntry() override;

  void onExit() override;

private:
  // this client behavior has its own temporal publisher to control the robot
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_;

  // we store the options that customize the behavior
  CbSpiralMotionOptions options_;
};
}  // namespace cl_nav2z
