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

#include <smacc2/smacc_state.hpp>
#include <smacc2/smacc_state_machine.hpp>

namespace smacc2
{
std::string ISmaccState::getClassName() { return demangleSymbol(typeid(*this).name()); }

void ISmaccState::notifyTransitionFromTransitionTypeInfo(TypeInfo::Ptr & transitionType)
{
  RCLCPP_INFO_STREAM(getLogger(), "TRANSITION RULE TRIGGERED: " << transitionType->getFullName());

  //auto currstateinfo = this->getStateMachine().getCurrentStateInfo();
  auto currstateinfo = this->stateInfo_;

  if (currstateinfo != nullptr)
  {
    //RCLCPP_ERROR_STREAM(getLogger(),"CURRENT STATE INFO: " << currstateinfo->fullStateName);
    for (auto & transition : currstateinfo->transitions_)
    {
      std::string transitionCandidateName = transition.transitionTypeInfo->getFullName();
      //RCLCPP_ERROR_STREAM(getLogger(),"candidate transition: " << transitionCandidateName);

      if (transitionType->getFullName() == transitionCandidateName)
      {
        this->getStateMachine().publishTransition(transition);
        return;
      }
    }

    // debug information if not found
    RCLCPP_ERROR_STREAM(
      getLogger(),
      "Transition happened, from current state "
        << currstateinfo->getDemangledFullName()
        << " but there is not any transitioninfo match available to publish transition: "
        << transitionType->getFullName());
    std::stringstream ss;

    auto stateinfo = currstateinfo;

    for (auto & transition : currstateinfo->transitions_)
    {
      std::string transitionCandidateName = transition.transitionTypeInfo->getFullName();
      RCLCPP_ERROR_STREAM(getLogger(), "- candidate transition: " << transitionCandidateName);
    }

    RCLCPP_ERROR(getLogger(), "Ancestors candidates: ");

    std::list<const SmaccStateInfo *> ancestors;
    stateinfo->getAncestors(ancestors);

    for (auto & ancestor : ancestors)
    {
      RCLCPP_ERROR_STREAM(getLogger(), " * Ancestor " << ancestor->getDemangledFullName() << ":");
      for (auto & transition : ancestor->transitions_)
      {
        std::string transitionCandidateName = transition.transitionTypeInfo->getFullName();
        RCLCPP_ERROR_STREAM(getLogger(), "- candidate transition: " << transitionCandidateName);
        if (transitionType->getFullName() == transitionCandidateName)
        {
          RCLCPP_ERROR(getLogger(), "GOTCHA");
        }
      }
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(
      getLogger(), "Transition happened, but current state was not set. Transition candidates:");
  }
}

}  // namespace smacc2
