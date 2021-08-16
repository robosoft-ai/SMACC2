#include <smacc/common.h>
#include <smacc/introspection/introspection.h>

namespace smacc
{
using namespace smacc::introspection;

std::map<const std::type_info *, std::vector<ClientBehaviorInfoEntry>>
  SmaccStateInfo::staticBehaviorInfo;
std::map<
  const std::type_info *, std::vector<std::shared_ptr<smacc::introspection::SmaccStateReactorInfo>>>
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

}  // namespace smacc
