#pragma once

#include <smacc2/client_bases/smacc_http_client.hpp>
#include <smacc2/smacc.hpp>

namespace sm_atomic_http {

template <typename TSource, typename TOrthogonal>
struct EvHttp : sc::event<EvHttp<TSource, TOrthogonal>> {};

class CbHttpRequest : public smacc2::SmaccClientBehavior {
 public:
  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation() {
    triggerTranstition = []() {
      auto event = new EvHttp<TSourceObject, TOrthogonal>();
      this->postEvent(event);
    }
  }

  void runtimeConfigure() override {
    this->requiresClient(cl_http_);
    cl_http->onResponseReceived(&CbHttpRequest::onResponseReceived, this);
  }

  void onResponseReceived() { triggerTranstition(); }

  void onEntry() override {
    RCLCPP_INFO(getLogger(), "On Entry!");

    cl_http_->makeRequest();
  }

  void onExit() override { RCLCPP_INFO(getLogger(), "Cb on exit!"); }

 private:
  smacc2::client_bases::SmaccHttpClient* cl_http_;

  std::function<void(const std::string&)> triggerTranstition;
};
}  // namespace sm_atomic_http
