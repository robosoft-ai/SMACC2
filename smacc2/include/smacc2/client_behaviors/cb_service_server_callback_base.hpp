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
#include <smacc2/smacc_client_behavior.hpp>

namespace smacc2
{
template <typename TService>
class CbServiceServerCallbackBase : public smacc2::SmaccClientBehavior
{
public:
  virtual void onEntry() override
  {
    this->requiresClient(attachedClient_);
    attachedClient_->onServiceRequestReceived(
      &CbServiceServerCallbackBase::onServiceRequestReceived, this);
  }

  virtual void onServiceRequestReceived(
    const std::shared_ptr<typename TService::Request> req,
    std::shared_ptr<typename TService::Response> res) = 0;

protected:
  smacc2::client_bases::SmaccServiceServerClient<TService> * attachedClient_ = nullptr;
};
}  // namespace smacc2
