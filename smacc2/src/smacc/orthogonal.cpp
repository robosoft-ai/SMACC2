#include <smacc/impl/smacc_state_machine_impl.h>
#include <smacc/smacc_client_behavior.h>
#include <smacc/smacc_orthogonal.h>
#include <smacc/trace_provider.h>

#define statename stateMachine_->getCurrentState()->getClassName().c_str()
#define orthogonalName getName().c_str()
#define cbName clBehavior->getName().c_str()

namespace smacc
{
void ISmaccOrthogonal::setStateMachine(ISmaccStateMachine *value)
{
  this->stateMachine_ = value;
  this->onInitialize();
  this->initializeClients();
}

void ISmaccOrthogonal::initializeClients()
{
  for (auto &c : this->clients_)
  {
    c->initialize();
  }
}

rclcpp::Node::SharedPtr ISmaccOrthogonal::getNode()
{
  return this->stateMachine_->getNode();
}

void ISmaccOrthogonal::addClientBehavior(std::shared_ptr<smacc::ISmaccClientBehavior> clBehavior)
{
  if (clBehavior != nullptr)
  {
    RCLCPP_INFO(getNode()->get_logger(), "[Orthogonal %s] adding client behavior: %s", this->getName().c_str(),
                clBehavior->getName().c_str());
    clBehavior->stateMachine_ = this->getStateMachine();
    clBehavior->currentOrthogonal = this;

    clientBehaviors_.push_back(clBehavior);
  }
  else
  {
    RCLCPP_INFO(getNode()->get_logger(), "[orthogonal %s] no client behaviors in this state", this->getName().c_str());
  }
}

void ISmaccOrthogonal::onInitialize()
{
}

std::string ISmaccOrthogonal::getName() const
{
  return demangleSymbol(typeid(*this).name());
}

void ISmaccOrthogonal::runtimeConfigure()
{
  for (auto &clBehavior : clientBehaviors_)
  {
    RCLCPP_INFO(getNode()->get_logger(), "[Orthogonal %s] runtimeConfigure, current Behavior: %s",
                this->getName().c_str(), clBehavior->getName().c_str());

    clBehavior->runtimeConfigure();
  }
}

void ISmaccOrthogonal::onEntry()
{
  if (clientBehaviors_.size() > 0)
  {
    for (auto &clBehavior : clientBehaviors_)
    {
      RCLCPP_INFO(getNode()->get_logger(), "[Orthogonal %s] OnEntry, current Behavior: %s", orthogonalName, cbName);

      try
      {
        tracepoint(smacc_trace, client_behavior_on_entry_start, statename, orthogonalName, cbName);
        clBehavior->executeOnEntry();
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(getNode()->get_logger(),
                     "[ClientBehavior %s] Exception on Entry - continuing with next client behavior. Exception info: "
                     "%s",
                     cbName, e.what());
      }
      tracepoint(smacc_trace, client_behavior_on_entry_end, statename, orthogonalName, cbName);
    }
  }
  else
  {
    RCLCPP_INFO(getNode()->get_logger(), "[Orthogonal %s] OnEntry -> empty orthogonal (no client behavior) ",
                orthogonalName);
  }
}

void ISmaccOrthogonal::onExit()
{
  if (clientBehaviors_.size() > 0)
  {
    for (auto &clBehavior : clientBehaviors_)
    {
      RCLCPP_INFO(getNode()->get_logger(), "[Orthogonal %s] OnExit, current Behavior: %s", orthogonalName, cbName);
      try
      {
        tracepoint(smacc_trace, client_behavior_on_exit_start, statename, orthogonalName, cbName);
        clBehavior->executeOnExit();
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(getNode()->get_logger(),
                     "[ClientBehavior %s] Exception onExit - continuing with next client behavior. Exception info: %s",
                     cbName, e.what());
      }
      tracepoint(smacc_trace, client_behavior_on_exit_end, statename, orthogonalName, cbName);
    }

    int i = 0;
    for (auto &clBehavior : clientBehaviors_)
    {
      clBehavior->dispose();
      clientBehaviors_[i] = nullptr;
    }

    clientBehaviors_.clear();
  }
  else
  {
    RCLCPP_INFO(getNode()->get_logger(), "[Orthogonal %s] OnExit", orthogonalName);
  }
}
}  // namespace smacc