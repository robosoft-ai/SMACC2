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

#include "orthogonals/or_navigation.hpp"
using namespace smacc2;

namespace sm_aws_warehouse_navigation
{
//STATES
class StState1;
class StState2;

//STATE_MACHINE
struct SmAwsWarehouseNavigation
: public smacc2::SmaccStateMachineBase<SmAwsWarehouseNavigation, StState1>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  virtual void onInitialize() override { this->createOrthogonal<OrNavigation>(); }
};

}  // namespace sm_aws_warehouse_navigation

#include "states/st_state_1.hpp"
#include "states/st_state_2.hpp"
