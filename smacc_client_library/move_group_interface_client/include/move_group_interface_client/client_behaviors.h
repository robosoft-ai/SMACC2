#pragma once

// BASIC MANIPULATION BEHAVIORS
#include "client_behaviors/cb_move_end_effector.h"
#include "client_behaviors/cb_move_end_effector_relative.h"
#include "client_behaviors/cb_move_cartesian_relative.h"
#include "client_behaviors/cb_move_joints.h"
#include "client_behaviors/cb_move_known_state.h"
#include "client_behaviors/cb_move_named_target.h"

// ADVANCED MANIPULATION BEHAVIORS
#include "client_behaviors/cb_move_end_effector_trajectory.h"
#include "client_behaviors/cb_circular_pivot_motion.h"
#include "client_behaviors/cb_end_effector_rotate.h"
#include "client_behaviors/cb_pouring_motion.h"
#include "client_behaviors/cb_move_cartesian_relative2.h"

// HISTORY BASED BEHAVIRORS
#include "client_behaviors/cb_move_last_trajectory_initial_state.h"
#include "client_behaviors/cb_execute_last_trajectory.h"
#include "client_behaviors/cb_undo_last_trajectory.h"

// GRASPING BEHAVIORS
#include "client_behaviors/cb_attach_object.h"
#include "client_behaviors/cb_detach_object.h"