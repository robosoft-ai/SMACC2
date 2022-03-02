// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <smacc2/introspection/introspection.hpp>
#include "rclcpp/rclcpp.hpp"

namespace smacc2
{
namespace introspection
{
rclcpp::Node::SharedPtr globalNh_;

void transitionInfoToMsg(
  const SmaccTransitionInfo & transition, smacc2_msgs::msg::SmaccTransition & transitionMsg)
{
  transitionMsg.index = transition.index;

  if (transition.sourceState != nullptr)
  {
    transitionMsg.source_state_name = transition.sourceState->demangledStateName;
  }

  transitionMsg.transition_name = transition.transitionTag;
  transitionMsg.transition_type = transition.transitionType;

  if (transition.eventInfo != nullptr)
  {
    transitionMsg.event.event_type = transition.eventInfo->getEventTypeName();
    transitionMsg.event.event_source = transition.eventInfo->getEventSourceName();
    transitionMsg.event.event_object_tag = transition.eventInfo->getOrthogonalName();
    transitionMsg.event.label = transition.eventInfo->label;
  }

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
}  // namespace introspection
}  // namespace smacc2
