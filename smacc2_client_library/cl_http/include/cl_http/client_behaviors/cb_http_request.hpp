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
 * 	 Authors: Jaycee Lock & Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <cl_http/cl_http.hpp>
#include <cl_http/components/cp_http_request_executor.hpp>
#include <cstring>
#include <smacc2/smacc.hpp>

namespace cl_http
{

class CbHttpRequestBase : public smacc2::SmaccClientBehavior
{
public:
  CbHttpRequestBase(const CpHttpRequestExecutor::HttpMethod http_request_type)
  : kRequestType(http_request_type)
  {
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    // Base implementation - can be overridden by derived classes
  }

  virtual void runtimeConfigure() override
  {
    // Access executor component and connect to response signal
    this->requiresComponent(requestExecutor_);
    this->getStateMachine()->createSignalConnection(
      requestExecutor_->onResponseReceived_, &CbHttpRequestBase::onResponseReceived, this);
  }

  virtual void onResponseReceived(const CpHttpRequestExecutor::TResponse & /*response*/) {}

  virtual void onEntry() override { requestExecutor_->executeRequest(kRequestType); }

  virtual void onExit() override {}

private:
  const CpHttpRequestExecutor::HttpMethod kRequestType;

  CpHttpRequestExecutor * requestExecutor_;
};
}  // namespace cl_http
