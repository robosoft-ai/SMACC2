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
#include <smacc2/client_bases/smacc_action_client_base.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace smacc2
{
namespace client_behaviors
{
using namespace smacc2::client_bases;

// waits the action server is available in the current orthogonal
class CbWaitActionServer : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbWaitActionServer(std::chrono::milliseconds timeout);
  virtual ~CbWaitActionServer();

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
    this->requiresClient(client_);
  }

  void executeOnEntry() override;

private:
  ISmaccActionClient * client_;
  std::chrono::milliseconds timeout_;
};
}  // namespace client_behaviors
}  // namespace smacc2
