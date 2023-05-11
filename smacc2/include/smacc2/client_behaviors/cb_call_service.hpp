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
#include <smacc2/impl/smacc_asynchronous_client_behavior_impl.hpp>
#include <smacc2/smacc_client_behavior.hpp>

namespace smacc2
{
namespace client_behaviors
{
using namespace std::chrono_literals;
template <typename ServiceType>
class CbServiceCall : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbServiceCall(const char * serviceName) : serviceName_(serviceName)
  {
    request_ = std::make_shared<typename ServiceType::Request>();
    pollRate_ = 100ms;
  }

  CbServiceCall(
    const char * serviceName, std::shared_ptr<typename ServiceType::Request> request,
    std::chrono::milliseconds pollRate = 100ms)
  : serviceName_(serviceName), request_(request), result_(nullptr), pollRate_(pollRate)
  {
  }

  void onEntry() override
  {
    RCLCPP_DEBUG_STREAM(
      getLogger(), "[" << this->getName() << "] creating ros service client: " << serviceName_);

    client_ = getNode()->create_client<ServiceType>(serviceName_);

    RCLCPP_DEBUG_STREAM(
      getLogger(), "[" << this->getName() << "] making service request to " << serviceName_);

    resultFuture_ = client_->async_send_request(request_).future.share();

    std::future_status status = resultFuture_.wait_for(0s);

    RCLCPP_DEBUG_STREAM(
      getLogger(), "thread state: " << (int)status << " ok " << rclcpp::ok() << " shutdown "
                                    << this->isShutdownRequested() << "");
    while (status != std::future_status::ready && rclcpp::ok() && !this->isShutdownRequested())
    {
      RCLCPP_DEBUG_STREAM(getLogger(), "[" << this->getName() << "] waiting response ");
      rclcpp::sleep_for(pollRate_);
      status = resultFuture_.wait_for(0s);
    }

    if (status == std::future_status::ready)
    {
      RCLCPP_DEBUG_STREAM(getLogger(), "[" << this->getName() << "] response received ");
      result_ = resultFuture_.get();
      onServiceResponse(result_);
      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_DEBUG_STREAM(getLogger(), "[" << this->getName() << "] response not received ");
      this->postFailureEvent();
    }
  }

  std::shared_future<std::shared_ptr<typename ServiceType::Response>> resultFuture_;

  typename std::shared_ptr<typename ServiceType::Response> result_;
  std::chrono::milliseconds pollRate_;

protected:
  //rclcpp::NodeHandle nh_;
  std::shared_ptr<rclcpp::Client<ServiceType>> client_;
  std::string serviceName_;
  std::shared_ptr<typename ServiceType::Request> request_;

  virtual void onServiceResponse(std::shared_ptr<typename ServiceType::Response> /*result*/)
  {
    RCLCPP_DEBUG_STREAM(getLogger(), "[" << this->getName() << "] response received ");
  }
};

}  // namespace client_behaviors
}  // namespace smacc2
