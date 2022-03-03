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

#include <rcl/time.h>
#include <chrono>
#include <functional>
#include <smacc2/client_bases/smacc_action_client.hpp>
#include <smacc2/smacc_orthogonal.hpp>
#include <smacc2/smacc_signal_detector.hpp>
#include <smacc2/smacc_state_machine.hpp>
#include <smacc2_msgs/msg/smacc_status.hpp>
#include <smacc2_msgs/msg/smacc_transition_log_entry.hpp>

namespace smacc2
{
using namespace std::chrono_literals;
using namespace smacc2::introspection;
ISmaccStateMachine::ISmaccStateMachine(
  std::string stateMachineName, SignalDetector * signalDetector)
: currentState_(nullptr), stateSeqCounter_(0)
{
  rclcpp::NodeOptions node_options;
  // This enables loading arbitrary parameters
  // However, best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  // TODO(henningkayser): remove once all parameters are declared inside the components
  // node_options.automatically_declare_parameters_from_overrides(true);

  nh_ = rclcpp::Node::make_shared(stateMachineName, node_options);  //
  RCLCPP_INFO_STREAM(
    nh_->get_logger(), "Creating State Machine Base: " << nh_->get_fully_qualified_name());

  signalDetector_ = signalDetector;
  signalDetector_->initialize(this);

  std::string runMode;
  if (nh_->get_parameter("run_mode", runMode))
  {
    if (runMode == "debug")
    {
      runMode_ = SMRunMode::DEBUG;
    }
    else if (runMode == "release")
    {
      runMode_ = SMRunMode::RELEASE;
    }
    else
    {
      RCLCPP_ERROR(nh_->get_logger(), "Incorrect run_mode value: %s", runMode.c_str());
    }
  }
  else
  {
    runMode_ = SMRunMode::DEBUG;
  }
}

ISmaccStateMachine::~ISmaccStateMachine()
{
  RCLCPP_INFO(nh_->get_logger(), "Finishing State Machine");
}

rclcpp::Node::SharedPtr ISmaccStateMachine::getNode() { return this->nh_; }

void ISmaccStateMachine::reset() {}

void ISmaccStateMachine::stop() {}

void ISmaccStateMachine::eStop() {}

const std::map<std::string, std::shared_ptr<smacc2::ISmaccOrthogonal>> &
ISmaccStateMachine::getOrthogonals() const
{
  return this->orthogonals_;
}

void ISmaccStateMachine::updateStatusMessage()
{
  std::lock_guard<std::recursive_mutex> lock(m_mutex_);

  if (currentStateInfo_ != nullptr)
  {
    RCLCPP_WARN_STREAM(
      nh_->get_logger(), "[StateMachine] setting state active "
                           << ": " << currentStateInfo_->getFullPath());

    if (runMode_ == SMRunMode::DEBUG)
    {
      status_msg_.current_states.clear();
      std::list<const SmaccStateInfo *> ancestorList;
      currentStateInfo_->getAncestors(ancestorList);

      for (auto & ancestor : ancestorList)
      {
        status_msg_.current_states.push_back(ancestor->toShortName());
      }

      status_msg_.global_variable_names.clear();
      status_msg_.global_variable_values.clear();

      for (auto entry : this->globalData_)
      {
        status_msg_.global_variable_names.push_back(entry.first);
        status_msg_.global_variable_values.push_back(
          entry.second.first());  // <- invoke to_string()
      }

      status_msg_.header.stamp = this->nh_->now();
      stateMachineStatusPub_->publish(status_msg_);
    }
  }
}

void ISmaccStateMachine::publishTransition(const SmaccTransitionInfo & transitionInfo)
{
  smacc2_msgs::msg::SmaccTransitionLogEntry transitionLogEntry;
  transitionLogEntry.timestamp = this->nh_->now();
  transitionInfoToMsg(transitionInfo, transitionLogEntry.transition);
  this->transitionLogHistory_.push_back(transitionLogEntry);

  transitionLogPub_->publish(transitionLogEntry);
}

void ISmaccStateMachine::onInitialize() {}

void ISmaccStateMachine::onInitialized()
{
  auto ros_clock = rclcpp::Clock::make_shared();
  timer_ =
    rclcpp::create_timer(nh_, ros_clock, 0.5s, [=]() { this->state_machine_visualization(); });
}

void ISmaccStateMachine::initializeROS(std::string shortname)
{
  RCLCPP_WARN_STREAM(nh_->get_logger(), "State machine base creation:" << shortname);
  // STATE MACHINE TOPICS
  stateMachinePub_ = nh_->create_publisher<smacc2_msgs::msg::SmaccStateMachine>(
    shortname + "/smacc/state_machine_description", 1);
  stateMachineStatusPub_ =
    nh_->create_publisher<smacc2_msgs::msg::SmaccStatus>(shortname + "/smacc/status", 1);
  transitionLogPub_ = nh_->create_publisher<smacc2_msgs::msg::SmaccTransitionLogEntry>(
    shortname + "/smacc/transition_log", 1);

  // STATE MACHINE SERVICES
  transitionHistoryService_ = nh_->create_service<smacc2_msgs::srv::SmaccGetTransitionHistory>(
    shortname + "/smacc/transition_log_history",
    std::bind(
      &ISmaccStateMachine::getTransitionLogHistory, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3));
}

void ISmaccStateMachine::getTransitionLogHistory(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<smacc2_msgs::srv::SmaccGetTransitionHistory::Request> /*req*/,
  std::shared_ptr<smacc2_msgs::srv::SmaccGetTransitionHistory::Response> res)
{
  RCLCPP_WARN(
    nh_->get_logger(), "Requesting Transition Log History, current size: %ld",
    this->transitionLogHistory_.size());
  res->history = this->transitionLogHistory_;
}

void ISmaccStateMachine::state_machine_visualization()
{
  std::lock_guard<std::recursive_mutex> lock(m_mutex_);

  smacc2_msgs::msg::SmaccStateMachine state_machine_msg;
  state_machine_msg.states = stateMachineInfo_->stateMsgs;

  std::sort(
    state_machine_msg.states.begin(), state_machine_msg.states.end(),
    [](auto & a, auto & b) { return a.index < b.index; });
  stateMachinePub_->publish(state_machine_msg);

  status_msg_.header.stamp = this->nh_->now();
  stateMachineStatusPub_->publish(status_msg_);
}

// void ISmaccStateMachine::lockStateMachine(std::string msg)
// {
//   RCLCPP_DEBUG(nh_->get_logger(), "-- locking SM: %s", msg.c_str());
//   m_mutex_.lock();
// }

// void ISmaccStateMachine::unlockStateMachine(std::string msg)
// {
//   RCLCPP_DEBUG(nh_->get_logger(), "-- unlocking SM: %s", msg.c_str());
//   m_mutex_.unlock();
// }

std::string ISmaccStateMachine::getStateMachineName()
{
  return demangleSymbol(typeid(*this).name());
}

void ISmaccStateMachine::disposeStateAndDisconnectSignals()
{
  for (auto & conn : this->stateCallbackConnections)
  {
    RCLCPP_WARN_STREAM(
      getLogger(),
      "[StateMachine] Disconnecting scoped-lifetime SmaccSignal "
      "subscription");
    conn.disconnect();
  }

  this->stateCallbackConnections.clear();

  currentState_ = nullptr;
}

void ISmaccStateMachine::checkStateMachineConsistence()
{
  // transition from an orthogonal that doesn’t exist.
  // transition from a source that doesn’t exist.

  // std::stringstream errorbuffer;
  // bool errorFound = false;

  // for (auto &stentry : this->stateMachineInfo_->states)
  // {
  //     auto stinfo = stentry.second;

  //     for (auto &transition : stinfo->transitions_)
  //     {
  //         auto evinfo = transition.eventInfo;
  //         bool found = false;
  //         for (auto &orthogonal : orthogonals_)
  //         {
  //             if (orthogonal.first == evinfo->getOrthogonalName())
  //             {
  //                 found = true;
  //                 break;
  //             }
  //         }

  //         if (!found)
  //         {
  //             errorbuffer << "---------" << std::endl
  //                         << "[Consistency Checking] Transition event refers not existing orthogonal." << std::endl
  //                         << "State: " << demangleType(*stinfo->tid_) << std::endl
  //                         << "Transition: " << transition.transitionTypeInfo->getFullName() << std::endl
  //                         << "Orthogonal: " << evinfo->getOrthogonalName() << std::endl
  //                         << "---------" << std::endl;

  //             errorFound = true;
  //         }
  //         //std::string getEventSourceName();
  //         //std::string getOrthogonalName();
  //     }
  // }

  // if (errorFound)
  // {
  //     RCLCPP_WARN_STREAM(nh_->get_logger(),"== STATE MACHINE CONSISTENCY CHECK: ==" << std::endl
  //                                                              << errorbuffer.str() << std::endl
  //                                                              << "=================");
  // }
  // cb from a client that doesn’t exist – don’t worry about making clients dynamically.
}

}  // namespace smacc2
