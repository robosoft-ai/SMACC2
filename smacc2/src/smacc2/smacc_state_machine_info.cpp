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

#include <smacc2/introspection/introspection.hpp>
#include <smacc2/smacc_state_machine.hpp>

namespace smacc2
{
void SmaccStateMachineInfo::assembleSMStructureMessage(ISmaccStateMachine * sm)
{
  RCLCPP_INFO(getLogger(), "----------- PRINT STATE MACHINE STRUCTURE -------------------");
  stateMsgs.clear();
  for (auto & val : this->states)
  {
    smacc2_msgs::msg::SmaccState stateMsg;
    auto state = val.second;
    stateMsg.index = state->stateIndex_;

    std::stringstream ss;
    ss << "**** State: " << demangleSymbol(val.first.c_str()) << std::endl;

    stateMsg.name = state->getDemangledFullName();
    stateMsg.level = (int)state->getStateLevel();

    ss << "**** State: " << stateMsg.name << std::endl;

    ss << "Index: " << stateMsg.index << std::endl;
    ss << "StateLevel: " << stateMsg.level << std::endl;

    ss << " Childstates:" << std::endl;

    for (auto & child : state->children_)
    {
      auto childStateName = child->getDemangledFullName();
      stateMsg.children_states.push_back(childStateName);

      ss << " - " << childStateName << std::endl;
    }

    ss << " Transitions:" << std::endl;

    for (auto & transition : state->transitions_)
    {
      smacc2_msgs::msg::SmaccTransition transitionMsg;

      transitionInfoToMsg(transition, transitionMsg);

      ss << " - Transition.  " << std::endl;
      ss << "      - Index: " << transitionMsg.index << std::endl;
      ss << "      - Transition Name: " << transitionMsg.transition_name << std::endl;
      ss << "      - Transition Type: " << transitionMsg.transition_type << std::endl;
      ss << "      - Event Type :" << transitionMsg.event.event_type << std::endl;
      ss << "      - Event Source: " << transitionMsg.event.event_source << std::endl;
      ss << "      - Event ObjectTag: " << transitionMsg.event.event_object_tag << std::endl;
      ss << "      - Event Label: " << transitionMsg.event.label << std::endl;
      ss << "      - Destiny State: " << transitionMsg.destiny_state_name << std::endl;
      ss << "      - Owner State: " << transitionMsg.destiny_state_name << std::endl;
      ss << "      - Is History Node: " << std::to_string(transitionMsg.history_node) << std::endl;
      ss << "      - TransitionC++Type: " << transition.transitionTypeInfo->getFullName()
         << std::endl;
      ss << "      - EventC++Type: " << transition.eventInfo->eventType->getFullName() << std::endl;

      stateMsg.transitions.push_back(transitionMsg);
    }

    const std::type_info * statetid = state->tid_;

    std::map<const std::type_info *, std::vector<smacc2::ClientBehaviorInfoEntry *>>
      smaccBehaviorInfoMappingByOrthogonalType;

    ss << " Orthogonals:" << std::endl;
    if (SmaccStateInfo::staticBehaviorInfo.count(statetid) > 0)
    {
      for (auto & bhinfo : SmaccStateInfo::staticBehaviorInfo[statetid])
      {
        if (smaccBehaviorInfoMappingByOrthogonalType.count(bhinfo.orthogonalType) == 0)
        {
          smaccBehaviorInfoMappingByOrthogonalType[bhinfo.orthogonalType] =
            std::vector<smacc2::ClientBehaviorInfoEntry *>();
        }

        smaccBehaviorInfoMappingByOrthogonalType[bhinfo.orthogonalType].push_back(&bhinfo);
      }
    }

    auto & runtimeOrthogonals = sm->getOrthogonals();

    for (auto & orthogonal : runtimeOrthogonals)
    {
      smacc2_msgs::msg::SmaccOrthogonal orthogonalMsg;

      const auto * orthogonaltid = &typeid(*(orthogonal.second));
      orthogonalMsg.name = demangleSymbol(orthogonaltid->name());

      ss << " - orthogonal: " << orthogonalMsg.name << std::endl;

      if (smaccBehaviorInfoMappingByOrthogonalType[orthogonaltid].size() > 0)
      {
        auto & behaviors = smaccBehaviorInfoMappingByOrthogonalType[orthogonaltid];
        for (auto & bhinfo : behaviors)
        {
          auto ClientBehaviorName = demangleSymbol(bhinfo->behaviorType->name());
          orthogonalMsg.client_behavior_names.push_back(ClientBehaviorName);
          ss << "          - client behavior: " << ClientBehaviorName << std::endl;
        }
      }
      else
      {
        ss << "          - NO Client BEHAVIORS -" << std::endl;
      }

      auto & clients = orthogonal.second->getClients();
      if (clients.size() > 0)
      {
        for (auto & client : clients)
        {
          auto clientTid = client->getType();
          auto clientName = clientTid->getNonTemplatedTypeName();
          orthogonalMsg.client_names.push_back(clientName);
          ss << "          - client: " << clientName << std::endl;
        }
      }
      else
      {
        ss << "          - NO CLIENTS - " << std::endl;
      }
      stateMsg.orthogonals.push_back(orthogonalMsg);
    }

    ss << " State event generators:" << std::endl;
    if (SmaccStateInfo::eventGeneratorsInfo.count(statetid) > 0)
    {
      int k = 0;
      for (auto & eginfo : SmaccStateInfo::eventGeneratorsInfo[statetid])
      {
        smacc2_msgs::msg::SmaccEventGenerator eventGeneratorMsg;
        eventGeneratorMsg.index = k++;
        eventGeneratorMsg.type_name = eginfo->eventGeneratorType->getFullName();

        ss << " - event generator: " << eventGeneratorMsg.type_name << std::endl;
        if (eginfo->objectTagType != nullptr)
        {
          eventGeneratorMsg.object_tag = eginfo->objectTagType->getFullName();
          ss << "        - object tag: " << eventGeneratorMsg.object_tag << std::endl;
        }
        stateMsg.event_generators.push_back(eventGeneratorMsg);
      }
    }

    ss << " State reactors:" << std::endl;
    if (SmaccStateInfo::stateReactorsInfo.count(statetid) > 0)
    {
      int k = 0;
      for (auto & srinfo : SmaccStateInfo::stateReactorsInfo[statetid])
      {
        smacc2_msgs::msg::SmaccStateReactor stateReactorMsg;
        stateReactorMsg.index = k++;
        stateReactorMsg.type_name = srinfo->stateReactorType->getFullName();

        ss << " - state reactor: " << stateReactorMsg.type_name << std::endl;
        if (srinfo->objectTagType != nullptr)
        {
          stateReactorMsg.object_tag = srinfo->objectTagType->getFullName();
          ss << "        - object tag: " << stateReactorMsg.object_tag << std::endl;
        }

        for (auto & tev : srinfo->sourceEventTypes)
        {
          // WE SHOULD CREATE A SMACC_EVENT_INFO TYPE, also using in typewalker transition
          auto eventTypeName = tev->getEventTypeName();
          smacc2_msgs::msg::SmaccEvent event;

          ss << "             - triggering event: " << tev->getEventTypeName() << std::endl;
          event.event_type = eventTypeName;

          event.event_source = tev->getEventSourceName();
          ss << "                 - source type: " << event.event_source << std::endl;

          event.event_object_tag = tev->getOrthogonalName();
          ss << "                 - source object: " << event.event_object_tag << std::endl;

          event.label = tev->label;
          ss << "                 - event label: " << event.label << std::endl;

          stateReactorMsg.event_sources.push_back(event);
        }

        stateMsg.state_reactors.push_back(stateReactorMsg);
      }
    }
    else
    {
      ss << "- NO STATE REACTORS - " << std::endl;
    }

    RCLCPP_INFO_STREAM(getLogger(), ss.str());
    stateMsgs.push_back(stateMsg);
  }
}
}  // namespace smacc2
