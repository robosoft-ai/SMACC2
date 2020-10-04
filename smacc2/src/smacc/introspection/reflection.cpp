#include <smacc/introspection/introspection.h>
#include "rclcpp/rclcpp.hpp"

namespace smacc
{
namespace introspection
{
rclcpp::Node::SharedPtr globalNh_;

void transitionInfoToMsg(const SmaccTransitionInfo &transition, smacc_msgs::msg::SmaccTransition &transitionMsg)
{
    transitionMsg.index = transition.index;
    transitionMsg.event.event_type = transition.eventInfo->getEventTypeName();

    transitionMsg.source_state_name = transition.sourceState->demangledStateName;

    transitionMsg.transition_name = transition.transitionTag;
    transitionMsg.transition_type = transition.transitionType;
    transitionMsg.event.event_source = transition.eventInfo->getEventSourceName();
    transitionMsg.event.event_object_tag = transition.eventInfo->getOrthogonalName();
    transitionMsg.event.label = transition.eventInfo->label;
    transitionMsg.history_node = transition.historyNode;

    if (transition.historyNode)
    {
        if (transition.destinyState->parentState_ != nullptr)
        {
            transitionMsg.destiny_state_name = transition.destinyState->parentState_->demangledStateName;
        }
        else
        {
            transitionMsg.destiny_state_name = "";
        }
    }
    else
    {
        transitionMsg.destiny_state_name = transition.destinyState->demangledStateName;
    }
}
} // namespace introspection
} // namespace smacc