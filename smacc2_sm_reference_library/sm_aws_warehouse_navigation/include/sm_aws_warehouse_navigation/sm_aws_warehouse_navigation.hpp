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

#include <smacc2/smacc.hpp>

// CLIENTS
#include <nav2z_client/nav2z_client.hpp>

// CLIENT BEHAVIORS
#include <nav2z_client/client_behaviors.hpp>
#include <smacc2/client_behaviors/cb_ros_launch.hpp>
#include <smacc2/client_behaviors/cb_wait_action_server.hpp>

using namespace cl_nav2z;
using namespace smacc2::client_behaviors;

// ORTHOGONALS
#include "orthogonals/or_navigation.hpp"
using namespace smacc2;

namespace sm_aws_warehouse_navigation
{
//STATES
struct StAcquireSensors;
struct StStartNavigation;
struct StInitialNavigateForward;
struct StRotateMainAisle;
struct StCheckPoint1;

//STATE_MACHINE
struct SmAwsWarehouseNavigation
: public smacc2::SmaccStateMachineBase<SmAwsWarehouseNavigation, StAcquireSensors>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override { this->createOrthogonal<OrNavigation>(); }
};

}  // namespace sm_aws_warehouse_navigation

#include "states/st_acquire_sensors.hpp"
#include "states/st_start_navigation.hpp"
#include "states/st_checkpoint_1.hpp"
#include "states/st_initial_forward.hpp"
#include "states/st_rotate_main_aisle.hpp"
