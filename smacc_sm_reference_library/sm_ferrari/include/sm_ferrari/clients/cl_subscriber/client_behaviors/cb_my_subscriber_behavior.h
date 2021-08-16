
#pragma once
#include <sm_ferrari/clients/cl_subscriber/cl_subscriber.h>

namespace sm_ferrari
{
namespace cl_subscriber
{
template <typename TSource, typename TOrthogonal>
struct EvMyBehavior : sc::event<EvMyBehavior<TSource, TOrthogonal>>
{
};

class CbMySubscriberBehavior : public smacc::SmaccClientBehavior
{
public:
  void onEntry()
  {
    ClSubscriber * client;
    this->requiresClient(client);

    client->onMessageReceived(&CbMySubscriberBehavior::onMessageReceived, this);
  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    postMyEvent_ = [=] { this->postEvent<EvMyBehavior<TSourceObject, TOrthogonal>>(); };
  }

  void onMessageReceived(const std_msgs::msg::Float32 & msg)
  {
    if (msg.data > 30)
    {
      RCLCPP_INFO(
        getNode()->get_logger(),
        "[CbMySubscriberBehavior] received message from topic with value > 30. "
        "Posting event!");
      this->postMyEvent_();
    }
  }

  std::function<void()> postMyEvent_;
};
}  // namespace cl_subscriber
}  // namespace sm_ferrari