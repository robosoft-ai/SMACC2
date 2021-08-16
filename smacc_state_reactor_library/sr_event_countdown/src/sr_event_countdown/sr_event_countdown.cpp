
#include <smacc/common.h>
#include <sr_event_countdown/sr_event_countdown.h>
#include <memory>

namespace smacc
{
namespace state_reactors
{
using namespace smacc::introspection;
SrEventCountdown::SrEventCountdown(int eventCount) : eventCount_(eventCount) {}

void SrEventCountdown::onInitialized()
{
  for (auto type : eventTypes)
  {
    triggeredEvents[type] = false;
  }
}

void SrEventCountdown::onEventNotified(const std::type_info * eventType)
{
  eventCount_--;
  RCLCPP_INFO_STREAM(
    getNode()->get_logger(), "SB COUNTDOWN (" << eventCount_ << ") RECEIVED EVENT OF TYPE:"
                                              << demangleSymbol(eventType->name()));

  // RCLCPP_INFO_STREAM(getNode()->get_logger(),"SB ALL RECEIVED EVENT OF TYPE:" << demangleSymbol(eventType->name()));
  // triggeredEvents[eventType] = true;

  // for (auto &entry : triggeredEvents)
  // {
  //     RCLCPP_INFO_STREAM(getNode()->get_logger(),demangleSymbol(entry.first->name()) << " = " << entry.second);
  // }
}

bool SrEventCountdown::triggers()
{
  if (eventCount_ == 0)
  {
    RCLCPP_INFO_STREAM(getNode()->get_logger(), "SB COUNTDOWN (" << eventCount_ << ") TRIGGERS!");
    return true;
  }
  else
  {
    return false;
  }

  // RCLCPP_INFO(getNode()->get_logger(),"SB All TRIGGERS?");
  // for (auto &entry : triggeredEvents)
  // {
  //     if (!entry.second)
  //         return false;
  // }
  // RCLCPP_INFO(getNode()->get_logger(),"SB ALL TRIGGERED");
  // return true;
}

}  // namespace state_reactors
}  // namespace smacc
