/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#pragma once

#include <smacc/smacc_client.h>
#include <optional>

namespace smacc
{
namespace client_bases
{
template <typename ServiceType>
class SmaccServiceClient : public smacc::ISmaccClient
{
public:
  std::optional<std::string> serviceName_;

  SmaccServiceClient() { initialized_ = false; }

  virtual void onInitialize() override
  {
    if (!initialized_)
    {
      if (!serviceName_)
      {
        RCLCPP_ERROR(getNode()->get_logger(), "service client with no service name set. Skipping.");
      }
      else
      {
        RCLCPP_INFO_STREAM(
          getNode()->get_logger(), "[" << this->getName() << "] Client Service: " << *serviceName_);
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
}  // namespace smacc