// Copyright 2023 RobosoftAI Inc.
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
 * 	 Authors: Jaycee Lock
 *
 ******************************************************************************************************************/

#pragma once

#include <cstring>
#include <http_client/cl_http_client.hpp>
#include <smacc2/smacc.hpp>

namespace cl_http
{

class CbHttpRequestBase : public smacc2::SmaccClientBehavior
{
public:
  CbHttpRequestBase(const ClHttp::kHttpRequestMethod http_request_type)
  : kRequestType(http_request_type)
  {
  }

  template <typename TOrthogonal, typename TSourceObject>
  [[deprecated(
    "Use onStateOrthogonalAllocation instead. onOrthogonalAllocation will be removed in future "
    "versions.")]] void
  onOrthogonalAllocation()
  {
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    // Base implementation - can be overridden by derived classes
  }

  virtual void runtimeConfigure() override
  {
    this->requiresClient(cl_http_);
    cl_http_->onResponseReceived(&CbHttpRequestBase::onResponseReceived, this);
  }

  virtual void onResponseReceived(const ClHttp::TResponse & response) {}

  virtual void onEntry() override { cl_http_->makeRequest(kRequestType); }

  virtual void onExit() override {}

private:
  const ClHttp::kHttpRequestMethod kRequestType;

  ClHttp * cl_http_;
};
}  // namespace cl_http
