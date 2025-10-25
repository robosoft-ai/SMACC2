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
 *****************************************************************************************************************/
#pragma once

// BASIC MANIPULATION BEHAVIORSz
#include "client_behaviors/cb_move_cartesian_relative.hpp"
#include "client_behaviors/cb_move_end_effector.hpp"
#include "client_behaviors/cb_move_end_effector_relative.hpp"
#include "client_behaviors/cb_move_joints.hpp"
#include "client_behaviors/cb_move_known_state.hpp"
#include "client_behaviors/cb_move_named_target.hpp"

// ADVANCED MANIPULATION BEHAVIORS
#include "client_behaviors/cb_circular_pivot_motion.hpp"
#include "client_behaviors/cb_end_effector_rotate.hpp"
#include "client_behaviors/cb_move_cartesian_relative2.hpp"
#include "client_behaviors/cb_move_end_effector_trajectory.hpp"
#include "client_behaviors/cb_pouring_motion.hpp"

// HISTORY BASED BEHAVIRORS
#include "client_behaviors/cb_execute_last_trajectory.hpp"
#include "client_behaviors/cb_move_last_trajectory_initial_state.hpp"
#include "client_behaviors/cb_undo_last_trajectory.hpp"

// GRASPING BEHAVIORS
#include "client_behaviors/cb_attach_object.hpp"
#include "client_behaviors/cb_detach_object.hpp"
