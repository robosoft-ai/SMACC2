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

#pragma once

#include <optional>
#include <smacc2/smacc_client.hpp>
#include <smacc2/smacc_signal.hpp>

namespace smacc2
{
namespace client_bases
{
template <typename TService>
class SmaccServiceServerClient : public smacc2::ISmaccClient
{
  using TServiceRequest = typename TService::Request;
  using TServiceResponse = typename TService::Response;

public:
  std::optional<std::string> serviceName_;
  SmaccServiceServerClient() { initialized_ = false; }
  SmaccServiceServerClient(std::string service_name)
  {
    serviceName_ = service_name;
    initialized_ = false;
  }

  virtual ~SmaccServiceServerClient() {}

  smacc2::SmaccSignal<void(
    const std::shared_ptr<typename TService::Request>,
    std::shared_ptr<typename TService::Response>)>
    onServiceRequestReceived_;

  template <typename T>
  boost::signals2::connection onServiceRequestReceived(
    void (T::*callback)(
      const std::shared_ptr<typename TService::Request>,
      std::shared_ptr<typename TService::Response>),
    T * object)
  {
    return this->getStateMachine()->createSignalConnection(
      onServiceRequestReceived_, callback, object);
  }

  void onInitialize() override
  {
    if (!initialized_)
    {
      if (!serviceName_)
      {
        RCLCPP_ERROR_STREAM(
          getLogger(),
          "[" << this->getName() << "] service server with no service name set. Skipping.");
      }
      else
      {
        RCLCPP_INFO_STREAM(
          getLogger(), "[" << this->getName() << "] Client Service: " << *serviceName_);

        server_ = getNode()->create_service<TService>(
          *serviceName_, std::bind(
                           &SmaccServiceServerClient<TService>::serviceCallback, this,
                           std::placeholders::_1, std::placeholders::_2));

        this->initialized_ = true;
      }
    }
  }

private:
  void serviceCallback(
    const std::shared_ptr<typename TService::Request> req,
    std::shared_ptr<typename TService::Response> res)
  {
    onServiceRequestReceived_(req, res);
  }
  typename rclcpp::Service<TService>::SharedPtr server_;
  bool initialized_;
};
}  // namespace client_bases
}  // namespace smacc2
