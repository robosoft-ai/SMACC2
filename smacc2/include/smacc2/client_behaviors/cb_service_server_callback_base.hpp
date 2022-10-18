#pragma once
#include <smacc2/smacc_client_behavior.hpp>

namespace smacc2 {
template <typename TService>
class CbServiceServerCallbackBase : public smacc2::SmaccClientBehavior {
 public:
  virtual void onEntry() override {
    this->requiresClient(attachedClient_);
    attachedClient_->onServiceRequestReceived(
        &CbServiceServerCallbackBase::onServiceRequestReceived, this);
  }

  virtual void onServiceRequestReceived(const std::shared_ptr<typename TService::Request>  req,
                                        std::shared_ptr<typename TService::Response> res) = 0;

 protected:
  smacc2::client_bases::SmaccServiceServerClient<TService>* attachedClient_ =
      nullptr;
};
}  // namespace smacc
