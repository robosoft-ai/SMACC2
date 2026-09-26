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

#include <cl_px4_mr/client_behaviors/cb_wait_for_heading_stable.hpp>
#include <cl_px4_mr/components/cp_vehicle_local_position.hpp>

#include <chrono>
#include <deque>
#include <utility>

namespace cl_px4_mr
{

namespace
{
float wrapPi(float a)
{
  while (a > M_PI) a -= 2.0f * M_PI;
  while (a < -M_PI) a += 2.0f * M_PI;
  return a;
}
}  // namespace

CbWaitForHeadingStable::CbWaitForHeadingStable(HeadingStableParams params) : params_(params) {}

void CbWaitForHeadingStable::onEntry()
{
  if (localPosition_ == nullptr)
  {
    RCLCPP_ERROR(getLogger(), "CbWaitForHeadingStable: no CpVehicleLocalPosition");
    this->postPx4Failure();
    return;
  }
  const bool checkExpected = std::isfinite(params_.expectedHeadingRad);
  RCLCPP_INFO(
    getLogger(),
    "CbWaitForHeadingStable: waiting for the heading to hold within %.2f deg/s over %.0f s%s "
    "(timeout %.0f s)",
    params_.maxDriftRadS * 180.0 / M_PI, params_.windowS,
    checkExpected ? " and match the parked heading" : "", params_.timeoutS);

  using clock = std::chrono::steady_clock;
  const auto start = clock::now();
  std::deque<std::pair<double, float>> samples;  // (t since start, heading)
  rclcpp::Rate rate(10.0);
  double lastReport = -10.0;

  while (!this->isShutdownRequested())
  {
    const double t = std::chrono::duration<double>(clock::now() - start).count();
    if (localPosition_->isValid())
    {
      samples.emplace_back(t, localPosition_->getHeading());
    }
    while (!samples.empty() && t - samples.front().first > params_.windowS)
    {
      samples.pop_front();
    }

    bool windowFull =
      !samples.empty() && samples.back().first - samples.front().first >= 0.9 * params_.windowS;
    float drift = 0.0f, offset = 0.0f;
    bool stable = false, aligned = true;
    if (windowFull)
    {
      // unwrap over the window: max excursion from the first sample
      float lo = 0.0f, hi = 0.0f;
      for (const auto & s : samples)
      {
        const float d = wrapPi(s.second - samples.front().second);
        lo = std::min(lo, d);
        hi = std::max(hi, d);
      }
      drift = hi - lo;
      stable = drift <= static_cast<float>(params_.maxDriftRadS * params_.windowS);
      if (checkExpected)
      {
        offset = wrapPi(samples.back().second - params_.expectedHeadingRad);
        aligned = std::fabs(offset) <= params_.headingTolRad;
      }
    }

    if (windowFull && stable && aligned)
    {
      RCLCPP_INFO(
        getLogger(),
        "CbWaitForHeadingStable: heading %.1f deg stable (%.2f deg over %.0f s%s) - posting "
        "success",
        samples.back().second * 180.0 / M_PI, drift * 180.0 / M_PI, params_.windowS,
        checkExpected ? (", offset " + std::to_string(offset * 180.0 / M_PI) + " deg").c_str()
                      : "");
      this->postPx4Success();
      return;
    }

    if (t - lastReport >= 2.0)
    {
      lastReport = t;
      if (!windowFull)
      {
        RCLCPP_INFO(
          getLogger(), "CbWaitForHeadingStable: collecting (%zu samples, position valid=%d)",
          samples.size(), static_cast<int>(localPosition_->isValid()));
      }
      else
      {
        RCLCPP_WARN(
          getLogger(),
          "CbWaitForHeadingStable: heading %.1f deg NOT ready: drift %.2f deg/s%s - the EKF "
          "yaw is not trustworthy for takeoff",
          samples.back().second * 180.0 / M_PI, drift / params_.windowS * 180.0 / M_PI,
          checkExpected
            ? (", offset from parked heading " + std::to_string(offset * 180.0 / M_PI) + " deg")
                .c_str()
            : "");
      }
    }

    if (t > params_.timeoutS)
    {
      RCLCPP_ERROR(
        getLogger(),
        "CbWaitForHeadingStable: timeout (%.0f s): heading %.1f deg, drift %.2f deg/s. "
        "Refusing to arm; restart the estimator / SITL",
        params_.timeoutS, samples.empty() ? 0.0 : samples.back().second * 180.0 / M_PI,
        drift / params_.windowS * 180.0 / M_PI);
      this->postPx4Failure();
      return;
    }
    rate.sleep();
  }
  RCLCPP_WARN(getLogger(), "CbWaitForHeadingStable: shutdown requested");
  this->postPx4Failure();
}

}  // namespace cl_px4_mr
