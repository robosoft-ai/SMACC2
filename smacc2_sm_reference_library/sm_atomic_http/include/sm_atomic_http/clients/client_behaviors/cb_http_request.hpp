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

#include <http_client/http_client.hpp>
#include <smacc2/smacc.hpp>

#include <cstring>

namespace sm_atomic_http {

template <typename TSource, typename TOrthogonal>
struct EvHttp : sc::event<EvHttp<TSource, TOrthogonal>> {};

class CbHttpRequest : public smacc2::SmaccClientBehavior {
 public:
  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation() {
    triggerTranstition = [this]() {
      auto event = new EvHttp<TSourceObject, TOrthogonal>();
      this->postEvent(event);
    };
  }

  void runtimeConfigure() override {
    this->requiresClient(cl_http_);
    cl_http_->onResponseReceived(&CbHttpRequest::onResponseReceived, this);
  }

  void onResponseReceived(const std::string& response) { triggerTranstition(); }

  void onEntry() override {
    RCLCPP_INFO(getLogger(), "On Entry!");

    cl_http_->makeRequest(
        cl_http::ClHttp::kHttpRequestMethod::GET);
  }

  void onExit() override { RCLCPP_INFO(getLogger(), "Cb on exit!"); }

 private:
  cl_http::ClHttp* cl_http_;

  std::function<void()> triggerTranstition;
};
}  // namespace sm_atomic_http
