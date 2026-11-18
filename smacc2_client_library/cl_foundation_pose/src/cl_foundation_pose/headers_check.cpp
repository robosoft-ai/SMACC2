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

// Includes every header of this header-only library, so that the build compiles them and clang-tidy
// reads them. Nothing from this file is installed.
#include <cl_foundation_pose/cl_foundation_pose.hpp>
#include <cl_foundation_pose/client_behaviors/cb_pause_object_tracking.hpp>
#include <cl_foundation_pose/client_behaviors/cb_track_object_pose.hpp>
#include <cl_foundation_pose/components/cp_object_tracker_1.hpp>
#include <cl_foundation_pose/components/cp_object_tracker_tf.hpp>
#include <cl_foundation_pose/components/tracker_utils.hpp>
