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

#include <math.h>

#include <smacc2/smacc.hpp>
#include <nav2z_client/components/pose/cp_pose.hpp>

namespace sm_dance_bot_warehouse_2
{
namespace cl_nav2z
{
using namespace std::chrono_literals;

// computes the distance to move forward inside a square region
// if the robot is located in the center of it
class CpSquareShapeBoundary : public smacc2::ISmaccComponent
{
public:
  float squareLenghtMeters_;
  ::cl_nav2z::Pose* robotPose_;

  CpSquareShapeBoundary(float squareLenghtMeters)
    : squareLenghtMeters_(squareLenghtMeters)
  {

  }

  void onInitialize() override
  {
    requiresComponent(robotPose_);
  }

  float getForwardDistance()
  {
    float currentAngle = robotPose_->getYaw();

    // warp 0 to 2pi
    while(currentAngle < 0)
    {
      currentAngle+=2*M_PI;
    }

    float side = squareLenghtMeters_/2.0;

    float fwdist = -1;
    float x,y;

    if((currentAngle > 0 && currentAngle < M_PI/4  )             // rightside
      || (currentAngle > 7*M_PI/4 && currentAngle <= 2*M_PI)     // rightside
      || (currentAngle > 3*M_PI/4 && currentAngle <= 5*M_PI/4)  // leftside
      )
    {
      float m = tan(currentAngle);
      x = side;
      y = m*x;

      fwdist = sqrt(x*x + y*y);
    }
    else if((currentAngle > M_PI/4 && currentAngle <= 3*M_PI/4)      // top side
            || (currentAngle > 5*M_PI/4 && currentAngle <= 7*M_PI/4)) // bottom side
    {
        float cotan = cos(currentAngle)/sin(currentAngle);
        y = side;
        x = cotan*y;

        fwdist = sqrt(x*x + y*y);
    }
    else
    {
      RCLCPP_FATAL_STREAM(getLogger(), "[" << getName() << " ] not implemented case");
    }

    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << " ] distance for yaw angle: " << currentAngle << " and square size: "<< squareLenghtMeters_ << " -- dist: " << fwdist <<"(x: "<< x <<" y:" << y <<")");

    return fwdist;
  }

};
}  // namespace cl_lidar
}  // namespace sm_dance_bot_strikes_back
