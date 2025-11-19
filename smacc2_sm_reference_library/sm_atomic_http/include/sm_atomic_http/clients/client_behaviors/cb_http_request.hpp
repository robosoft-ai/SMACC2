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
#include <http_client/client_behaviors/cb_http_get_request.hpp>
#include <smacc2/smacc.hpp>

namespace sm_atomic_http
{

template <typename TSource, typename TOrthogonal>
struct EvHttp : sc::event<EvHttp<TSource, TOrthogonal>>
{
};

class CbHttpRequest : public cl_http::CbHttpGetRequest
{
public:
  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    triggerTranstition = [this]()
    {
      auto event = new EvHttp<TSourceObject, TOrthogonal>();
      this->postEvent(event);
    };
  }

  void onResponseReceived(const cl_http::ClHttp::TResponse & response)
  {
    RCLCPP_INFO_STREAM(this->getLogger(), "ON RESPONSE");
    RCLCPP_INFO_STREAM(this->getLogger(), response.body());
    triggerTranstition();
  }

private:
  cl_http::ClHttp * cl_http_;

  std::function<void()> triggerTranstition;
};
}  // namespace sm_atomic_http
