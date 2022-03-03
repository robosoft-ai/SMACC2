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
template <typename ServiceType>
class CbServiceCall : public smacc2::SmaccAsyncClientBehavior
{
public:
  CbServiceCall(const char * serviceName) : serviceName_(serviceName) {}

  CbServiceCall(const char * serviceName, std::shared_ptr<typename ServiceType::Request>)
  : serviceName_(serviceName)
  {
  }

  void onEntry() override
  {
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << this->getName() << "] creating service client: " << serviceName_);

    client_ = getNode()->create_client<ServiceType>(serviceName_);

    result_ = client_->async_send_request(request_).get();

    //, std::bind(&CbServiceCall<ServiceType>::onServiceResponse, this, std::placeholders::_1));
  }

  typename std::shared_ptr<typename ServiceType::Response> result_;

protected:
  //rclcpp::NodeHandle nh_;
  std::shared_ptr<rclcpp::Client<ServiceType>> client_;
  std::string serviceName_;
  std::shared_ptr<typename ServiceType::Request> request_;

  void onServiceResponse(typename rclcpp::Client<ServiceType>::SharedFuture result)
  {
    result_ = result;
  }
};

}  // namespace client_behaviors
}  // namespace smacc2
