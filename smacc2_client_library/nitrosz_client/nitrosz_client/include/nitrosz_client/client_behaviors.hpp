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

#pragma once

// rotation behavior
#include <nitrosz_client/client_behaviors/cb_absolute_rotate.hpp>
#include <nitrosz_client/client_behaviors/cb_rotate.hpp>
#include <nitrosz_client/client_behaviors/cb_rotate_look_at.hpp>

#include <nitrosz_client/client_behaviors/cb_navigate_backwards.hpp>
#include <nitrosz_client/client_behaviors/cb_navigate_forward.hpp>
#include <nitrosz_client/client_behaviors/cb_navigate_global_position.hpp>
#include <nitrosz_client/client_behaviors/cb_retry_behavior.hpp>
#include <nitrosz_client/client_behaviors/cb_undo_path_backwards.hpp>

// nav2 synchronization behaviors
#include <nitrosz_client/client_behaviors/cb_wait_nav2_nodes.hpp>
#include <nitrosz_client/client_behaviors/cb_wait_pose.hpp>
#include <nitrosz_client/client_behaviors/cb_wait_transform.hpp>

// others
#include <nitrosz_client/client_behaviors/cb_abort_navigation.hpp>
#include <nitrosz_client/client_behaviors/cb_stop_navigation.hpp>

// waypoints behaviors
#include <nitrosz_client/client_behaviors/cb_navigate_named_waypoint.hpp>
#include <nitrosz_client/client_behaviors/cb_navigate_next_waypoint.hpp>
#include <nitrosz_client/client_behaviors/cb_navigate_next_waypoint_until_reached.hpp>
#include <nitrosz_client/client_behaviors/cb_seek_waypoint.hpp>

// slam behaviors
#include <nitrosz_client/client_behaviors/cb_pause_slam.hpp>
#include <nitrosz_client/client_behaviors/cb_resume_slam.hpp>
