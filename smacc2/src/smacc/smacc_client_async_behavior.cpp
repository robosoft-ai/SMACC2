#include <smacc/smacc_asynchronous_client_behavior.h>

using namespace std::chrono_literals;

namespace smacc
{
void SmaccAsyncClientBehavior::executeOnEntry()
{
  RCLCPP_INFO_STREAM(
    getNode()->get_logger(), "[" << getName() << "] Creating asynchronous onEntry thread");
  this->onEntryThread_ = std::async(std::launch::async, [=] {
    this->onEntry();
    this->postFinishEventFn_();
    return 0;
  });
}

void SmaccAsyncClientBehavior::waitFutureIfNotFinished(std::future<int> & threadfut)
{
  try
  {
    rclcpp::Rate r(100);
    while (rclcpp::ok())
    {
      bool valid = threadfut.valid();
      if (valid)
      {
        auto status = threadfut.wait_for(std::chrono::milliseconds(20));
        if (status == std::future_status::ready)
        {
          threadfut.get();
          break;
        }
      }

      r.sleep();
      // rclcpp::spin_some(getNode());
      RCLCPP_WARN_THROTTLE(
        getLogger(), *(getNode()->get_clock()), 1000,
        "[%s] waiting for finishing client behavior, before leaving the state. Is the client "
        "behavior stucked?",
        demangleType(typeid(*this)).c_str());
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_DEBUG(
      getNode()->get_logger(),
      "[SmaccAsyncClientBehavior] trying to join function, but it was alredy finished.");
  }
}

void SmaccAsyncClientBehavior::executeOnExit()
{
  RCLCPP_INFO_STREAM(
    getNode()->get_logger(), "[" << getName() << "] onExit - join async onEntry thread");

  waitFutureIfNotFinished(this->onEntryThread_);

  RCLCPP_INFO_STREAM(
    getNode()->get_logger(), "[" << getName() << "] onExit - Creating asynchronous onExit thread");
  this->onExitThread_ = std::async(std::launch::async, [=] {
    this->onExit();
    return 0;
  });
}

void SmaccAsyncClientBehavior::dispose()
{
  RCLCPP_DEBUG_STREAM(
    getNode()->get_logger(),
    "[" << getName()
        << "] Destroying client behavior- Waiting finishing of asynchronous onExit thread");
  try
  {
    this->onExitThread_.get();
  }
  catch (...)
  {
    RCLCPP_DEBUG(
      getNode()->get_logger(),
      "[SmaccAsyncClientBehavior] trying to Join onExit function, but it was alredy finished.");
  }

  RCLCPP_DEBUG_STREAM(
    getNode()->get_logger(),
    "[" << getName()
        << "] Destroying client behavior-  onExit thread finished. Proccedding destruction.");
}

SmaccAsyncClientBehavior::~SmaccAsyncClientBehavior() {}

void SmaccAsyncClientBehavior::postSuccessEvent() { postSuccessEventFn_(); }

void SmaccAsyncClientBehavior::postFailureEvent() { postFailureEventFn_(); }

}  // namespace smacc
