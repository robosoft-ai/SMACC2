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

#include <smacc2/common.hpp>
#include <smacc2/introspection/introspection.hpp>

namespace smacc2
{
using namespace smacc2::introspection;

std::map<const std::type_info *, std::vector<ClientBehaviorInfoEntry>>
  SmaccStateInfo::staticBehaviorInfo;
std::map<
  const std::type_info *,
  std::vector<std::shared_ptr<smacc2::introspection::SmaccStateReactorInfo>>>
  SmaccStateInfo::stateReactorsInfo;
std::map<const std::type_info *, std::vector<std::shared_ptr<SmaccEventGeneratorInfo>>>
  SmaccStateInfo::eventGeneratorsInfo;

SmaccStateInfo::SmaccStateInfo(
  const std::type_info * tid, std::shared_ptr<SmaccStateInfo> parentState,
  std::shared_ptr<SmaccStateMachineInfo> stateMachineInfo)
{
  tid_ = tid;
  parentState_ = parentState;
  stateMachine_ = stateMachineInfo;

  if (parentState_ != nullptr) depth_ = parentState->depth_ + 1;
}

void SmaccStateInfo::getAncestors(std::list<const SmaccStateInfo *> & ancestorsList) const
{
  ancestorsList.push_front(this);
  if (parentState_ != nullptr)
  {
    this->parentState_->getAncestors(ancestorsList);
  }
}

SmaccStateType SmaccStateInfo::getStateLevel()
{
  if (this->children_.size() == 0)
  {
    if (this->parentState_ != nullptr)
    {
      return SmaccStateType::SUPERSTATE_ROUTINE;
    }
    else
    {
      return SmaccStateType::STATE;
    }
  }
  else
  {
    return SmaccStateType::SUPERSTATE;
  }
}

rclcpp::Node::SharedPtr SmaccStateInfo::getNode() { return this->stateMachine_->getNode(); }

std::string SmaccStateInfo::getFullPath()
{
  if (parentState_ == nullptr)
    return this->toShortName();
  else
    return this->parentState_->getFullPath() + "/" + this->toShortName();
}

const std::string & SmaccStateInfo::toShortName() const { return this->demangledStateName; }

std::string SmaccStateInfo::getDemangledFullName() const
{
  return demangleSymbol(this->fullStateName.c_str());
}
//---------------------------------------------
SmaccEventInfo::SmaccEventInfo(std::shared_ptr<TypeInfo> eventType) { this->eventType = eventType; }

std::string SmaccEventInfo::getEventSourceName()
{
  if (eventType->templateParameters.size() > 0)
  {
    auto eventsourcename = demangleSymbol(eventType->templateParameters[0]->getFullName().c_str());
    return eventsourcename;
  }
  else
  {
    return "";
  }
}

std::string SmaccEventInfo::getEventTypeName()
{
  return demangleSymbol(eventType->getNonTemplatedTypeName().c_str());
}

std::string SmaccEventInfo::getOrthogonalName()
{
  if (eventType->templateParameters.size() > 1)
  {
    return demangleSymbol(eventType->templateParameters[1]->getFullName().c_str());
  }
  else
  {
    return "";
  }
}

}  // namespace smacc2
