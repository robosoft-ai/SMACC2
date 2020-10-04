#pragma once

#include <smacc/client_bases/smacc_subscriber_client.h>
#include <boost/statechart/event.hpp>

#include <ros/ros.h>
#include <ros/duration.h>
#include <boost/signals2.hpp>
#include <boost/optional/optional_io.hpp>

namespace cl_multirole_sensor
{
using namespace smacc;

template <typename TSource, typename TOrthogonal>
struct EvTopicMessageTimeout : sc::event<EvTopicMessageTimeout<TSource, TOrthogonal>>
{
  ros::TimerEvent timerData;
};

using namespace smacc::client_bases;

//---------------------------------------------------------------
template <typename MessageType>
class ClMultiroleSensor : public smacc::client_bases::SmaccSubscriberClient<MessageType>
{
public:
  typedef MessageType TMessageType;
  SmaccSignal<void(const ros::TimerEvent &)> onMessageTimeout_;

  ClMultiroleSensor()
      : smacc::client_bases::SmaccSubscriberClient<MessageType>()
  {
    //RCLCPP_INFO( getNode()->get_logger(),"[ClMultiroleSensor] constructor");
    initialized_ = false;
  }

  template <typename T>
  boost::signals2::connection onMessageTimeout(void (T::*callback)(const ros::TimerEvent &), T *object)
  {
    return this->getStateMachine()->createSignalConnection(onMessageTimeout_, callback, object);
  }

  std::function<void(const ros::TimerEvent &ev)> postTimeoutMessageEvent;

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    SmaccSubscriberClient<MessageType>::template onOrthogonalAllocation<TOrthogonal, TSourceObject>();

    this->postTimeoutMessageEvent = [=](auto &timerdata) {
      onMessageTimeout_(timerdata);

      auto event = new EvTopicMessageTimeout<TSourceObject, TOrthogonal>();
      event->timerData = timerdata;
      this->postEvent(event);
    };
  }

  virtual void initialize() override
  {
    if (!initialized_)
    {
      SmaccSubscriberClient<MessageType>::initialize();

      this->onMessageReceived(&ClMultiroleSensor<MessageType>::resetTimer, this);

      if (timeout_)
      {
        timeoutTimer_ = nh_.createTimer(*timeout_, boost::bind(&ClMultiroleSensor<MessageType>::timeoutCallback, this, _1));
        timeoutTimer_.start();
      }
      else
      {
        ROS_WARN("Timeout sensor client not set, skipping timeout watchdog funcionality");
      }

      initialized_ = true;
    }
  }

  boost::optional<ros::Duration> timeout_;

protected:
  void resetTimer(const MessageType &msg)
  {
    //reseting the timer
    timeoutTimer_.stop();
    timeoutTimer_.start();
  }

private:
  ros::Timer timeoutTimer_;
  bool initialized_;

  void timeoutCallback(const ros::TimerEvent &timerdata)
  {
    postTimeoutMessageEvent(timerdata);
  }
};
} // namespace cl_multirole_sensor