#pragma once

#include <smacc/client_bases/smacc_subscriber_client.h>
#include <smacc/smacc_signal.h>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace cl_multirole_sensor
{
using namespace smacc;

template <typename TSource, typename TOrthogonal>
struct EvTopicMessageTimeout : sc::event<EvTopicMessageTimeout<TSource, TOrthogonal>>
{
};

using namespace smacc::client_bases;

//---------------------------------------------------------------
template <typename MessageType>
class ClMultiroleSensor : public smacc::client_bases::SmaccSubscriberClient<MessageType>
{
public:
  typedef MessageType TMessageType;
  SmaccSignal<void()> onMessageTimeout_;

  ClMultiroleSensor() : smacc::client_bases::SmaccSubscriberClient<MessageType>()
  {
    //RCLCPP_INFO( getNode()->get_logger(),"[ClMultiroleSensor] constructor");
    initialized_ = false;
  }

  template <typename T>
  boost::signals2::connection onMessageTimeout(void (T::*callback)(), T * object)
  {
    return this->getStateMachine()->createSignalConnection(onMessageTimeout_, callback, object);
  }

  std::function<void()> postTimeoutMessageEvent;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    SmaccSubscriberClient<MessageType>::template onOrthogonalAllocation<
      TOrthogonal, TSourceObject>();

    this->postTimeoutMessageEvent = [=]() {
      onMessageTimeout_();

      auto event = new EvTopicMessageTimeout<TSourceObject, TOrthogonal>();
      this->postEvent(event);
    };
  }

  virtual void onInitialize() override
  {
    if (!initialized_)
    {
      SmaccSubscriberClient<MessageType>::onInitialize();

      this->onMessageReceived(&ClMultiroleSensor<MessageType>::resetTimer, this);

      if (timeout_)
      {
        auto ros_clock = rclcpp::Clock::make_shared();
        timeoutTimer_ = rclcpp::create_timer(
          this->getNode(), this->getNode()->get_clock(), *timeout_,
          std::bind(&ClMultiroleSensor<MessageType>::timeoutCallback, this));
        //timeoutTimer_->start();
        timeoutTimer_->reset();
      }
      else
      {
        RCLCPP_WARN(
          this->getNode()->get_logger(),
          "Timeout sensor client not set, skipping timeout watchdog funcionality");
      }

      initialized_ = true;
    }
  }

  std::optional<rclcpp::Duration> timeout_;

protected:
  void resetTimer(const MessageType & /*msg*/)
  {
    //reseting the timer
    timeoutTimer_->reset();
    //timeoutTimer_->stop();
    //timeoutTimer_->start();
  }

private:
  rclcpp::TimerBase::SharedPtr timeoutTimer_;
  bool initialized_;

  void timeoutCallback() { postTimeoutMessageEvent(); }
};
}  // namespace cl_multirole_sensor