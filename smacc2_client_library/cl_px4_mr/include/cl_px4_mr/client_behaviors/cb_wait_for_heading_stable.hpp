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

#pragma once

#include <cl_px4_mr/client_behaviors/cb_px4_client_behavior_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

#include <cmath>
#include <limits>

namespace cl_px4_mr
{

// Preflight gate on the EKF heading. A vehicle sitting still on the ground must
// report a constant heading; if the estimate is drifting, the yaw gyro bias
// has not converged (or the magnetometer has been rejected) and a takeoff in
// that state is unstable: with a large heading error the position loop pushes
// the wrong way and the vehicle flips over (sm_cl_px4_mr_test_5 run 19: 186
// degrees of drift in 30 s, crash 4 s after liftoff).
//
// Success when the heading moved less than maxDriftRadS * windowS over the
// last windowS seconds (and, if expectedHeadingRad is finite, is also within
// headingTolRad of it). Failure on timeout, with the drift rate in the log.
struct HeadingStableParams
{
  double maxDriftRadS = 0.0087;  // 0.5 deg/s
  double windowS = 5.0;
  double timeoutS = 60.0;
  // known heading of the parked vehicle (NED, rad); NaN = do not check
  float expectedHeadingRad = std::numeric_limits<float>::quiet_NaN();
  float headingTolRad = 0.26f;  // 15 deg
};

class CbWaitForHeadingStable : public CbPx4ClientBehaviorBase
{
public:
  explicit CbWaitForHeadingStable(HeadingStableParams params = {});

  void onEntry() override;
  void onExit() override {}

private:
  HeadingStableParams params_;
};

}  // namespace cl_px4_mr
