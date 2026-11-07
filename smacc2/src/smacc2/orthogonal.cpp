// Copyright 2025 Robosoft Inc.
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
  std::lock_guard<std::mutex> lock(this->mutex_);

  if (clBehavior != nullptr)
  {
    RCLCPP_INFO(
      getLogger(), "[Orthogonal: %s] Adding client behavior: %s.", this->getName().c_str(),
      clBehavior->getName().c_str());
    clBehavior->stateMachine_ = this->getStateMachine();
    clBehavior->currentOrthogonal = this;
    clBehavior->currentState = clBehavior->stateMachine_->getCurrentState();

    clientBehaviors_.back().push_back(clBehavior);
  }
  else
  {
    RCLCPP_INFO(
      getLogger(), "[Orthogonal: %s] No client behaviors in this state.", this->getName().c_str());
  }
}

void ISmaccOrthogonal::onInitialize() {}

void ISmaccOrthogonal::initState(ISmaccState * state)
{
  RCLCPP_INFO(
    getLogger(), "[Orthogonal: %s] InitState: %s.", this->getName().c_str(),
    state->getClassName().c_str());
  clientBehaviors_.push_back(std::vector<std::shared_ptr<smacc2::ISmaccClientBehavior>>());
}

std::string ISmaccOrthogonal::getName() const { return demangleSymbol(typeid(*this).name()); }

// Snapshot of the current state's behavior list, taken under mutex_. The callers iterate the
// copy with the mutex released, so behavior code never runs while this orthogonal's mutex is
// held (see the comment on mutex_ in the header). The copied shared_ptrs keep the behaviors
// alive for the duration of the loop; disposal runs on the same thread, after these loops.
std::vector<std::shared_ptr<smacc2::ISmaccClientBehavior>>
ISmaccOrthogonal::currentClientBehaviors()
{
  std::lock_guard<std::mutex> lock(this->mutex_);
  return clientBehaviors_.back();
}

void ISmaccOrthogonal::runtimeConfigure()
{
  const auto behaviors = this->currentClientBehaviors();

  for (auto & clBehavior : behaviors)
  {
    RCLCPP_INFO(
      getLogger(), "[Orthogonal: %s] runtimeConfigure(), current behavior: %s.",
      this->getName().c_str(), clBehavior->getName().c_str());

    clBehavior->runtimeConfigure();
  }
}

void ISmaccOrthogonal::onEntry()
{
  const auto behaviors = this->currentClientBehaviors();

  if (behaviors.size() > 0)
  {
    for (auto & clBehavior : behaviors)
    {
      RCLCPP_INFO(
        getLogger(), "[Orthogonal: %s] OnEntry, current Behavior: %s", orthogonalName, cbName);

      try
      {
        TRACETOOLS_TRACEPOINT(
          smacc2_client_behavior_on_entry_start, statename, orthogonalName, cbName);
        clBehavior->executeOnEntry();
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(
          getLogger(),
          "[ClientBehavior: %s] Exception on Entry - continuing with next client behavior. "
          "Exception info: "
          "%s",
          cbName, e.what());
      }
      TRACETOOLS_TRACEPOINT(smacc2_client_behavior_on_entry_end, statename, orthogonalName, cbName);
    }
  }
  else
  {
    RCLCPP_INFO(
      getLogger(), "[Orthogonal: %s] OnEntry -> empty orthogonal (no client behavior).",
      orthogonalName);
  }
}

void ISmaccOrthogonal::onExit()
{
  const auto behaviors = this->currentClientBehaviors();

  if (behaviors.size() > 0)
  {
    for (auto & clBehavior : behaviors)
    {
      RCLCPP_INFO(
        getLogger(), "[Orthogonal: %s] OnExit, current Behavior: %s", orthogonalName, cbName);
      try
      {
        TRACETOOLS_TRACEPOINT(
          smacc2_client_behavior_on_exit_start, statename, orthogonalName, cbName);
        clBehavior->executeOnExit();
      }
      catch (const std::exception & e)
      {
        RCLCPP_ERROR(
          getLogger(),
          "[ClientBehavior: %s] Exception onExit - continuing with next client behavior. Exception "
          "info: %s",
          cbName, e.what());
      }
      TRACETOOLS_TRACEPOINT(smacc2_client_behavior_on_exit_end, statename, orthogonalName, cbName);
    }
  }
  else
  {
    RCLCPP_INFO(getLogger(), "[Orthogonal %s] OnExit().", orthogonalName);
  }
}

void ISmaccOrthogonal::onDispose()
{
  for (auto & clBehavior : clientBehaviors_.back())
  {
    clBehavior->dispose();
    // No cast: the typed overload normalizes the key to the complete object's address.
    this->getStateMachine()->disconnectSmaccSignalObject(clBehavior.get());
  }

  clientBehaviors_.back().clear();
  clientBehaviors_.pop_back();
}

}  // namespace smacc2
