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
