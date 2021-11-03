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

// BASIC MANIPULATION BEHAVIORS
#include "client_behaviors/cb_move_cartesian_relative.h"
#include "client_behaviors/cb_move_end_effector.h"
#include "client_behaviors/cb_move_end_effector_relative.h"
#include "client_behaviors/cb_move_joints.h"
#include "client_behaviors/cb_move_known_state.h"
#include "client_behaviors/cb_move_named_target.h"

// ADVANCED MANIPULATION BEHAVIORS
#include "client_behaviors/cb_circular_pivot_motion.h"
#include "client_behaviors/cb_end_effector_rotate.h"
#include "client_behaviors/cb_move_cartesian_relative2.h"
#include "client_behaviors/cb_move_end_effector_trajectory.h"
#include "client_behaviors/cb_pouring_motion.h"

// HISTORY BASED BEHAVIRORS
#include "client_behaviors/cb_execute_last_trajectory.h"
#include "client_behaviors/cb_move_last_trajectory_initial_state.h"
#include "client_behaviors/cb_undo_last_trajectory.h"

// GRASPING BEHAVIORS
#include "client_behaviors/cb_attach_object.h"
#include "client_behaviors/cb_detach_object.h"
