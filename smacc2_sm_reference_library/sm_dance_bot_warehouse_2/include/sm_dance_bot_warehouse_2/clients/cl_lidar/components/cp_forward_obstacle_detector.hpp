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

#include <smacc2/smacc.hpp>

namespace sm_dance_bot_warehouse_2
{
namespace cl_lidar
{
using namespace std::chrono_literals;

class CpForwardObstacleDetector : public smacc2::ISmaccComponent
{
public:
  sensor_msgs::msg::LaserScan lastScanMessage_;

  const float SECURITY_DISTANCE = 0.4;  // meters

  void onInitialize() override
  {
    auto client_ =
      dynamic_cast<smacc2::client_bases::SmaccSubscriberClient<sensor_msgs::msg::LaserScan> *>(
        owner_);
    client_->onMessageReceived(&CpForwardObstacleDetector::MessageCallbackStoreDistanceToWall, this);
  }

  int modulo_Euclidean(int a, int b)
  {
    int m = a % b;
    if (m < 0)
    {
      // m += (b < 0) ? -b : b; // avoid this form: -b is UB when b == INT_MIN
      m = (b < 0) ? m - b : m + b;
    }
    return m;
  }

  float getForwardDistance(int raysWidthCount = 0)
  {
    //auto fwdist = scanmsg.ranges[scanmsg.ranges.size() / 2] /*meter*/;
    auto fwdist = lastScanMessage_.ranges[0] /*meter*/;

//     RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] ranges[0]" << ss.str());

    std::stringstream ss;
    //check rays around main ray
    for (int i = 0; i < raysWidthCount; i++)
    {
      // int baseindex = 0;
      int scanindex =  (- raysWidthCount / 2 + i) % raysWidthCount;

      if(scanindex < 0)
      {
        scanindex = lastScanMessage_.ranges.size() + scanindex;
      }

      float fwdist2 = lastScanMessage_.ranges[scanindex];

      RCLCPP_INFO_STREAM(
        getLogger(), "[" << getName() << "]"
                         << "range[" << scanindex << "] = " << fwdist2);

      if (fwdist2 > 0.01 && fwdist2 < fwdist)
      {
        RCLCPP_INFO_STREAM(
          getLogger(), "[" << getName() << "]"
                           << "updated range[0] = " << fwdist2);

        fwdist = fwdist2;
      }
    }

    RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "]" << ss.str());

    /*if the distance is infinity or nan, use max range distance*/
    if (fwdist == std::numeric_limits<float>::infinity() || fwdist != fwdist)
    {
      fwdist = lastScanMessage_.range_max - SECURITY_DISTANCE /*meters*/;
      RCLCPP_INFO_THROTTLE(
        getLogger(), *(getNode()->get_clock()), 1000,
        "[CpForwardObstacleDetector] Distance to forward obstacle is not a number, setting default value "
        "to: %lf",
        lastScanMessage_.range_max);
    }
    else
    {
      RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] fwdist: " << fwdist);
      fwdist = std::max(fwdist - SECURITY_DISTANCE, 0.0F);
      RCLCPP_INFO_STREAM(getLogger(), "[" << getName() << "] fwdist: " << fwdist);
    }

    return fwdist;
  }

  void MessageCallbackStoreDistanceToWall(const sensor_msgs::msg::LaserScan & scanmsg)
  {
    this->lastScanMessage_ = scanmsg;
  }
};
}  // namespace cl_lidar
}  // namespace sm_dance_bot_warehouse_2
