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
 ******************************************************************************************************************/

#include <smacc2/smacc.hpp>

// CLIENTS
#include <ros_timer_client/cl_ros_timer.hpp>
#include <lifecyclenode_client/lifecyclenode_client.hpp>

//CLIENT BEHAVIORS
#include <ros_timer_client/client_behaviors/cb_timer_countdown_loop.hpp>
#include <ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp>
#include <lifecyclenode_client/client_behaviors.hpp>

// ORTHOGONALS
#include "orthogonals/or_timer.hpp"
#include "orthogonals/or_lifecyclenode.hpp"

using namespace boost;
using namespace smacc2;

namespace sm_atomic_lifecycle
{
//STATE
class StActivating;
class StActive;
class StCleaningUp;
class StConfiguring;
class StDeactivating;
class StErrorProcessing;
class StFinalized;
class StInactive;
class StShuttingDown;
class StUnconfigured;

//--------------------------------------------------------------------
//STATE_MACHINE
struct SmAtomicLifecycle : public smacc2::SmaccStateMachineBase<SmAtomicLifecycle, StUnconfigured>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override
  {
    this->createOrthogonal<OrTimer>();
    this->createOrthogonal<OrLifecycleNode>();
  }
};

}  // namespace sm_atomic_lifecycle

#include "states/st_activating.hpp"
#include "states/st_active.hpp"
#include "states/st_cleaning_up.hpp"
#include "states/st_configuring.hpp"
#include "states/st_deactivating.hpp"
#include "states/st_error_processing.hpp"
#include "states/st_finalized.hpp"
#include "states/st_inactive.hpp"
#include "states/st_shutting_down.hpp"
#include "states/st_unconfigured.hpp"
