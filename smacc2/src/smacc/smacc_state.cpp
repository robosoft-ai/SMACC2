#include <smacc/smacc_state.h>
#include <smacc/smacc_state_machine.h>

namespace smacc
{
std::string ISmaccState::getClassName() { return demangleSymbol(typeid(*this).name()); }

void ISmaccState::param(std::string param_name) { getNode()->declare_parameter(param_name); }

void ISmaccState::notifyTransitionFromTransitionTypeInfo(TypeInfo::Ptr & transitionType)
{
  RCLCPP_INFO_STREAM(
    getNode()->get_logger(), "NOTIFY TRANSITION: " << transitionType->getFullName());

  //auto currstateinfo = this->getStateMachine().getCurrentStateInfo();
  auto currstateinfo = this->stateInfo_;

  if (currstateinfo != nullptr)
  {
    //RCLCPP_ERROR_STREAM(getNode()->get_logger(),"CURRENT STATE INFO: " << currstateinfo->fullStateName);
    for (auto & transition : currstateinfo->transitions_)
    {
      std::string transitionCandidateName = transition.transitionTypeInfo->getFullName();
      //RCLCPP_ERROR_STREAM(getNode()->get_logger(),"candidate transition: " << transitionCandidateName);

      if (transitionType->getFullName() == transitionCandidateName)
      {
        this->getStateMachine().publishTransition(transition);
        return;
      }
    }

    // debug information if not found
    RCLCPP_ERROR_STREAM(
      getNode()->get_logger(),
      "Transition happened, from current state "
        << currstateinfo->getDemangledFullName()
        << " but there is not any transitioninfo match available to publish transition: "
        << transitionType->getFullName());
    std::stringstream ss;

    auto stateinfo = currstateinfo;

    for (auto & transition : currstateinfo->transitions_)
    {
      std::string transitionCandidateName = transition.transitionTypeInfo->getFullName();
      RCLCPP_ERROR_STREAM(
        getNode()->get_logger(), "- candidate transition: " << transitionCandidateName);
    }

    RCLCPP_ERROR(getNode()->get_logger(), "Ancestors candidates: ");

    std::list<const SmaccStateInfo *> ancestors;
    stateinfo->getAncestors(ancestors);

    for (auto & ancestor : ancestors)
    {
      RCLCPP_ERROR_STREAM(
        getNode()->get_logger(), " * Ancestor " << ancestor->getDemangledFullName() << ":");
      for (auto & transition : ancestor->transitions_)
      {
        std::string transitionCandidateName = transition.transitionTypeInfo->getFullName();
        RCLCPP_ERROR_STREAM(
          getNode()->get_logger(), "- candidate transition: " << transitionCandidateName);
        if (transitionType->getFullName() == transitionCandidateName)
        {
          RCLCPP_ERROR(getNode()->get_logger(), "GOTCHA");
        }
      }
    }
  }
  else
  {
    RCLCPP_ERROR_STREAM(
      getNode()->get_logger(),
      "Transition happened, but current state was not set. Transition candidates:");
  }
}

}  // namespace smacc