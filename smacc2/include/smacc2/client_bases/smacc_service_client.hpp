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

namespace smacc2
{
namespace client_bases
{
template <typename ServiceType>
class SmaccServiceClient : public smacc2::ISmaccClient
{
public:
  std::optional<std::string> serviceName_;

  SmaccServiceClient() { initialized_ = false; }

  void onInitialize() override
  {
    if (!initialized_)
    {
      if (!serviceName_)
      {
        RCLCPP_ERROR(getLogger(), "service client with no service name set. Skipping.");
      }
      else
      {
        RCLCPP_INFO_STREAM(
          getLogger(), "[" << this->getName() << "] Client Service: " << *serviceName_);
        this->initialized_ = true;
        client_ = getNode()->create_client<ServiceType>(*serviceName_);
      }
    }
  }

  std::shared_ptr<typename ServiceType::Response> call(
    std::shared_ptr<typename ServiceType::Request> & request)
  {
    auto result = client_->async_send_request(request);
    //rclcpp::spin_until_future_complete(getNode(), result);
    return result.get();
  }

protected:
  //rclcpp::NodeHandle nh_;
  std::shared_ptr<rclcpp::Client<ServiceType>> client_;
  bool initialized_;
};
}  // namespace client_bases
}  // namespace smacc2
