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

#include <smacc2/smacc.hpp>

#include <cl_keyboard/cl_keyboard.hpp>
#include <cl_nav2z/cl_nav2z.hpp>
#include <cl_ros2_timer/cl_ros2_timer.hpp>

#include <cl_keyboard/client_behaviors/cb_default_keyboard_behavior.hpp>
#include <cl_nav2z/client_behaviors/cb_abort_navigation.hpp>
#include <cl_nav2z/client_behaviors/cb_active_stop.hpp>
#include <cl_nav2z/client_behaviors/cb_load_waypoints_file.hpp>
#include <cl_nav2z/client_behaviors/cb_navigate_backwards.hpp>
#include <cl_nav2z/client_behaviors/cb_navigate_forward.hpp>
#include <cl_nav2z/client_behaviors/cb_navigate_named_waypoint.hpp>
#include <cl_nav2z/client_behaviors/cb_navigate_next_waypoint.hpp>
#include <cl_nav2z/client_behaviors/cb_navigate_next_waypoint_free.hpp>
#include <cl_nav2z/client_behaviors/cb_navigate_next_waypoint_until_reached.hpp>
#include <cl_nav2z/client_behaviors/cb_position_control_free_space.hpp>
#include <cl_nav2z/client_behaviors/cb_retry_behavior.hpp>
#include <cl_nav2z/client_behaviors/cb_rotate.hpp>
#include <cl_nav2z/client_behaviors/cb_rotate_look_at.hpp>
#include <cl_nav2z/client_behaviors/cb_seek_waypoint.hpp>
#include <cl_nav2z/client_behaviors/cb_stop_navigation.hpp>
#include <cl_nav2z/client_behaviors/cb_track_path_odometry.hpp>
#include <cl_nav2z/client_behaviors/cb_track_path_slam.hpp>
#include <cl_nav2z/client_behaviors/cb_wait_nav2_nodes.hpp>
#include <cl_nav2z/client_behaviors/cb_wait_pose.hpp>
#include <cl_nav2z/client_behaviors/cb_wait_transform.hpp>
#include <cl_ros2_timer/client_behaviors/cb_timer_countdown_once.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "orthogonals/or_keyboard.hpp"
#include "orthogonals/or_navigation.hpp"
#include "orthogonals/or_timer.hpp"

namespace sm_nav2_gazebo_test_4
{
using namespace std::chrono_literals;
using namespace boost;
using namespace smacc2;
using namespace smacc2::default_transition_tags;
using namespace cl_ros2_timer;
using namespace cl_keyboard;
using namespace cl_nav2z;

struct RETRY : SUCCESS
{
};

struct StAcquireSensors;
struct StSetInitialPose;
struct StWaitTransform;
struct StLoadWaypoints;
struct StNavigateWaypoints;
struct StRotateAtWaypoint;
struct StNavigateNamedWaypoint;
struct StSeekWaypoint;
struct StNavigateUntilReached;
struct StForwardCutShort;
struct StAbortNavigation;
struct StRetryForward;
struct StNavigateBackwards;
struct StRotateLookAt;
struct StPositionControl;
struct StNextWaypointFree;
struct StStopNavigation;
struct StActiveStop;
struct StFinalState;
struct StMissionFailed;

// One state per cl_nav2z behavior that the other Nav2 reference machines do not use; see README.md.
struct SmNav2GazeboTest4 : public smacc2::SmaccStateMachineBase<SmNav2GazeboTest4, StSetInitialPose>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrKeyboard>();
    this->createOrthogonal<OrNavigation>();
  }
};

}  // namespace sm_nav2_gazebo_test_4

#include "states/st_acquire_sensors.hpp"
#include "states/st_set_initial_pose.hpp"
#include "states/st_wait_transform.hpp"
#include "states/st_load_waypoints.hpp"
#include "states/st_navigate_waypoints.hpp"
#include "states/st_rotate_at_waypoint.hpp"
#include "states/st_navigate_named_waypoint.hpp"
#include "states/st_seek_waypoint.hpp"
#include "states/st_navigate_until_reached.hpp"
#include "states/st_forward_cut_short.hpp"
#include "states/st_abort_navigation.hpp"
#include "states/st_retry_forward.hpp"
#include "states/st_navigate_backwards.hpp"
#include "states/st_rotate_look_at.hpp"
#include "states/st_position_control.hpp"
#include "states/st_next_waypoint_free.hpp"
#include "states/st_stop_navigation.hpp"
#include "states/st_active_stop.hpp"
#include "states/st_final_state.hpp"
#include "states/st_mission_failed.hpp"
