// Copyright 2025 Robosoft Inc.
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

#include <cl_moveit2z/cl_moveit2z.hpp>

// Every other header of the library, so that this file compiles all of them and clang-tidy reads them.
#include <cl_moveit2z/client_behaviors.hpp>
#include <cl_moveit2z/client_behaviors/cb_attach_object.hpp>
#include <cl_moveit2z/client_behaviors/cb_circular_pivot_motion.hpp>
#include <cl_moveit2z/client_behaviors/cb_detach_object.hpp>
#include <cl_moveit2z/client_behaviors/cb_end_effector_rotate.hpp>
#include <cl_moveit2z/client_behaviors/cb_move_cartesian_relative2.hpp>
#include <cl_moveit2z/client_behaviors/cb_move_end_effector.hpp>
#include <cl_moveit2z/client_behaviors/cb_move_end_effector_trajectory.hpp>
#include <cl_moveit2z/client_behaviors/cb_move_joints.hpp>
#include <cl_moveit2z/client_behaviors/cb_move_known_state.hpp>
#include <cl_moveit2z/client_behaviors/cb_moveit2z_client_behavior_base.hpp>
#include <cl_moveit2z/client_behaviors/cb_undo_last_trajectory.hpp>
#include <cl_moveit2z/common.hpp>
#include <cl_moveit2z/components/cp_grasping_objects.hpp>
#include <cl_moveit2z/components/cp_joint_space_trajectory_planner.hpp>
#include <cl_moveit2z/components/cp_motion_planner.hpp>
#include <cl_moveit2z/components/cp_move_group_interface.hpp>
#include <cl_moveit2z/components/cp_tf_listener.hpp>
#include <cl_moveit2z/components/cp_trajectory_executor.hpp>
#include <cl_moveit2z/components/cp_trajectory_history.hpp>
#include <cl_moveit2z/components/cp_trajectory_visualizer.hpp>

namespace cl_moveit2z
{
// Declare the Client's constructors.
ClMoveit2z::ClMoveit2z(std::string groupName) : options_(groupName) {}

ClMoveit2z::ClMoveit2z(const moveit::planning_interface::MoveGroupInterface::Options & options)
: options_(options)
{
}

// Declare the Client's destructor.
ClMoveit2z::~ClMoveit2z() {}

}  // namespace cl_moveit2z
