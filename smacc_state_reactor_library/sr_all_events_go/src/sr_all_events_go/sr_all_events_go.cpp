
#include <sr_all_events_go/sr_all_events_go.h>
#include <smacc/common.h>

namespace smacc
{
namespace state_reactors
{

using namespace smacc::introspection;
void SrAllEventsGo::onInitialized()
{
    for (auto type : eventTypes)
    {
        triggeredEvents[type] = false;
    }
}

void SrAllEventsGo::onEventNotified(const std::type_info *eventType)
{
    RCLCPP_DEBUG_STREAM(getNode()->get_logger(), "[SB ALLEventsGo] RECEIVED EVENT OF TYPE:" << demangleSymbol(eventType->name()));
    triggeredEvents[eventType] = true;

    for (auto &entry : triggeredEvents)
    {
        RCLCPP_DEBUG_STREAM(getNode()->get_logger(), demangleSymbol(entry.first->name()) << " = " << entry.second);
    }
}

bool SrAllEventsGo::triggers()
{
    RCLCPP_DEBUG(getNode()->get_logger(), "SB All TRIGGERS?");
    for (auto &entry : triggeredEvents)
    {
        if (!entry.second)
            return false;
    }
    RCLCPP_DEBUG(getNode()->get_logger(), "SB ALL TRIGGERED");
    return true;
}
} // namespace state_reactors
} // namespace smacc