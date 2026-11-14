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

#pragma once

#include <example_interfaces/action/fibonacci.hpp>
#include <smacc2/client_core_components/cp_action_client.hpp>
#include <smacc2/smacc.hpp>

#include <sm_coretest_action_client_1/clients/cl_action.hpp>
#include <sm_coretest_action_client_1/components/cp_fibonacci_server.hpp>

namespace sm_coretest_action_client_1
{
class OrAction : public smacc2::Orthogonal<OrAction>
{
public:
  using ActionClient =
    smacc2::client_core_components::CpActionClient<example_interfaces::action::Fibonacci>;

  void onInitialize() override
  {
    auto client = this->createClient<ClAction>();
    client->createComponent<CpFibonacciServer>();
    // Two action client components of the same action type, told apart by name: one aimed at the
    // server above, one at an action nothing serves.
    client->createNamedComponent<ActionClient>("server", CpFibonacciServer::kActionName);
    client->createNamedComponent<ActionClient>("no_server", "/coretest_no_such_action");
  }
};
}  // namespace sm_coretest_action_client_1
