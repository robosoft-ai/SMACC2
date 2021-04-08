/*****************************************************************************************************************
 * ReelRobotix Inc. - Software License Agreement      Copyright (c) 2018-2021
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#pragma once
#include <smacc/smacc_client_behavior.h>

namespace smacc
{
template <typename TMsg>
class CbSubscriptionCallbackBase : public smacc::SmaccClientBehavior
{
public:
  virtual void onEntry() override
  {
    this->requiresClient(attachedClient_);
    attachedClient_->onMessageReceived(&CbSubscriptionCallbackBase::onMessageReceived, this);
  }

  virtual void onMessageReceived(const TMsg& msg) = 0;

protected:
  smacc::client_bases::SmaccSubscriberClient<TMsg>* attachedClient_ = nullptr;
};
}  // namespace smacc
