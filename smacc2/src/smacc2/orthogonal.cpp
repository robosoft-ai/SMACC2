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

#include <smacc2/impl/smacc_state_machine_impl.hpp>
#include <smacc2/smacc_client_behavior.hpp>
#include <smacc2/smacc_orthogonal.hpp>
#include <smacc2/smacc_tracing/smacc_tracing.hpp>

#define statename stateMachine_->getCurrentState()->getClassName().c_str()
#define orthogonalName getName().c_str()
#define cbName clBehavior->getName().c_str()

namespace smacc2
{
void ISmaccOrthogonal::setStateMachine(ISmaccStateMachine * value)
{
  this->stateMachine_ = value;
  this->onInitialize();
  this->initializeClients();
}

void ISmaccOrthogonal::initializeClients()
{
  for (auto & c : this->clients_)
  {
    c->initialize();
  }
}

rclcpp::Node::SharedPtr ISmaccOrthogonal::getNode() { return this->stateMachine_->getNode(); }

void ISmaccOrthogonal::addClientBehavior(std::shared_ptr<smacc2::ISmaccClientBehavior> clBehavior)
{
  if (clBehavior != nullptr)
  {
    RCLCPP_INFO(
      getLogger(), "[Orthogonal %s] adding client behavior: %s", this->getName().c_str(),
      clBehavior->getName().c_str());
    clBehavior->stateMachine_ = this->getStateMachine();
    clBehavior->currentOrthogonal = this;

    clientBehaviors_.push_back(clBehavior);
  }
  else
  {
    RCLCPP_INFO(
      getLogger(), "[orthogonal %s] no client behaviors in this state", this->getName().c_str());
  }
}

void ISmaccOrthogonal::onInitialize() {}

std::string ISmaccOrthogonal::getName() const { return demangleSymbol(typeid(*this).name()); }

void ISmaccOrthogonal::runtimeConfigure()
{
  for (auto & clBehavior : clientBehaviors_)
  {
    RCLCPP_INFO(
      getLogger(), "[Orthogonal %s] runtimeConfigure, current Behavior: %s",
      this->getName().c_str(), clBehavior->getName().c_str());

    clBehavior->runtimeConfigure();
  }
}

void ISmaccOrthogonal::onEntry()
{
  if (clientBehaviors_.size() > 0)
  {
    for (auto & clBehavior : clientBehaviors_)
    {
      RCLCPP_INFO(
        getLogger(), "[Orthogonal %s] OnEntry, current Behavior: %s", orthogonalName, cbName);

      try
      {
        TRACEPOINT(client_behavior_on_entry_start, statename, orthogonalName, cbName);
        clBehavior->executeOnEntry();
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(
          getLogger(),
          "[ClientBehavior %s] Exception on Entry - continuing with next client behavior. "
          "Exception info: "
          "%s",
          cbName, e.what());
      }
      TRACEPOINT(client_behavior_on_entry_end, statename, orthogonalName, cbName);
    }
  }
  else
  {
    RCLCPP_INFO(
      getLogger(), "[Orthogonal %s] OnEntry -> empty orthogonal (no client behavior) ",
      orthogonalName);
  }
}

void ISmaccOrthogonal::onExit()
{
  if (clientBehaviors_.size() > 0)
  {
    for (auto & clBehavior : clientBehaviors_)
    {
      RCLCPP_INFO(
        getLogger(), "[Orthogonal %s] OnExit, current Behavior: %s", orthogonalName, cbName);
      try
      {
        TRACEPOINT(client_behavior_on_exit_start, statename, orthogonalName, cbName);
        clBehavior->executeOnExit();
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(
          getLogger(),
          "[ClientBehavior %s] Exception onExit - continuing with next client behavior. Exception "
          "info: %s",
          cbName, e.what());
      }
      TRACEPOINT(client_behavior_on_exit_end, statename, orthogonalName, cbName);
    }

    int i = 0;
    for (auto & clBehavior : clientBehaviors_)
    {
      clBehavior->dispose();
      clientBehaviors_[i] = nullptr;
    }

    clientBehaviors_.clear();
  }
  else
  {
    RCLCPP_INFO(getLogger(), "[Orthogonal %s] OnExit", orthogonalName);
  }
}
}  // namespace smacc2
