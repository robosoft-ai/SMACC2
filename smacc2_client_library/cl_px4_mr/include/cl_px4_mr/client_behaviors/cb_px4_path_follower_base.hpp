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

#include <cl_px4_mr/client_behaviors/cb_px4_client_behavior_base.hpp>
#include <cl_px4_mr/utils/geo_utils.hpp>

#include <atomic>
#include <chrono>
#include <limits>
#include <vector>

namespace cl_px4_mr
{

enum class YawMode
{
  TANGENT,     // along the current path segment
  FIXED,       // PathFollowerParams::fixedYaw
  PER_VERTEX,  // NedPoint::yaw of the segment start (NaN -> tangent)
  HOLD_ENTRY   // heading at entry
};

struct PathFollowerParams
{
  float groundSpeed = 3.0f;  // m/s, carrot advance rate along the polyline
  // Max 3D distance the carrot may lead the vehicle. PX4 offboard position
  // mode flies at vel = MPC_XY_P (0.95) * (setpoint - position), saturated at
  // MPC_XY_VEL_MAX, so the steady-state lag is groundSpeed / 0.95 and the leash
  // must exceed that or it throttles the effective speed to ~0.95 * leash.
  float leash = 6.0f;
  float arrivalXyTol = 1.0f;  // final-vertex acceptance, metres
  float arrivalZTol = 0.5f;
  YawMode yawMode = YawMode::TANGENT;
  float fixedYaw = std::numeric_limits<float>::quiet_NaN();
  bool prependCurrentPosition = true;  // insert the entry position as vertex 0
  float minSegmentLength = 0.05f;      // drop degenerate segments
  // If the state machine armed no timeout: timeout = max(30 s, factor * length / speed).
  // 0 disables the auto-timeout.
  float autoTimeoutFactor = 2.5f;
};

// Streaming polyline follower. A derived behavior supplies buildPath() (a pure
// function of its params and the entry point); this base streams a moving
// position setpoint ("carrot") along that polyline at a commanded ground
// speed, throttled by a leash so the carrot never runs away from a lagging
// vehicle, and posts success once the carrot has reached the end AND the
// vehicle is within the arrival tolerances of the last vertex.
//
// It never uses CpGoalChecker (one global goal per client; closed paths would
// self-trigger it at entry). Completion is an update() predicate.
//
// Threading: components come from the inherited allocation (state machine
// thread). onEntry() (async thread) builds the path, arms the auto-timeout and
// commands vertex 0, then sets active_. update() (SignalDetector thread,
// ~20 Hz) only touches the path after observing active_. onExit() re-issues
// the last carrot so the offboard keep-alive holds there.
class CbPx4PathFollowerBase : public CbPx4ClientBehaviorBase
{
public:
  explicit CbPx4PathFollowerBase(PathFollowerParams params = {});
  virtual ~CbPx4PathFollowerBase() {}

  void setFollowerParams(const PathFollowerParams & params) { followerParams_ = params; }
  const PathFollowerParams & followerParams() const { return followerParams_; }

  void onEntry() override;
  void onExit() override;
  void update() override;

protected:
  // Called once from onEntry (async thread). Must depend only on params and
  // `current` (x, y, z, yaw = heading at entry). May adjust followerParams_
  // (e.g. yaw mode). Empty -> failure.
  virtual std::vector<NedPoint> buildPath(const NedPoint & current) = 0;

  // logging hooks
  virtual const char * behaviorName() const { return "CbPx4PathFollowerBase"; }
  virtual void onPathStarted(const std::vector<NedPoint> & /*path*/) {}
  virtual void onPathCompleted() {}

  float totalLength() const { return totalLen_; }
  float progressFraction() const { return totalLen_ > 0.0f ? sCarrot_ / totalLen_ : 1.0f; }

  PathFollowerParams followerParams_;

private:
  NedPoint commandFor(float s);
  float tangentYawAt(size_t segmentIndex) const;

  std::vector<NedPoint> path_;
  std::vector<float> cumLen_;
  float totalLen_ = 0.0f;
  float sCarrot_ = 0.0f;
  float entryHeading_ = 0.0f;
  NedPoint lastCmd_;
  int lastProgressDecile_ = -1;
  std::atomic<bool> active_{false};
  std::chrono::steady_clock::time_point lastUpdateTime_;
};

}  // namespace cl_px4_mr
