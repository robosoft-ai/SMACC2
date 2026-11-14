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

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/client_behaviors/cb_sequence.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace smacc2
{
namespace client_behaviors
{
CbSequence::CbSequence() {}

bool CbSequence::recursiveConsumeNext()
{
  // Starting a step creates a behavior and signal connections, which take the state machine's mutex.
  // An exiting state holds that mutex while it joins this thread, so it is never waited on here.
  std::unique_lock<std::recursive_mutex> lock(getStateMachine()->getMutex(), std::defer_lock);
  while (!lock.try_lock())
  {
    if (this->isShutdownRequested()) return false;
    rclcpp::sleep_for(std::chrono::milliseconds(1));
  }
  // Nor is a step started once ROS is shutting down and the behavior threads are being joined.
  if (this->isShutdownRequested() || !rclcpp::ok()) return false;

  RCLCPP_INFO(
    getLogger(), "[SequenceNode %p] next onEntry: %ld", static_cast<const void *>(this),
    sequenceNodes_.size());

  auto first = sequenceNodes_.front();
  auto onDelayedConfigureFn = first;

  RCLCPP_INFO(
    getLogger(), "[SequenceNode %p] Behavior on delayed sequence configure",
    static_cast<const void *>(this));
  bh_ = onDelayedConfigureFn();
  std::string currentNodeName = bh_->getName();

  RCLCPP_INFO(
    getLogger(), "[SequenceNode %p] Subscribing OnSuccess", static_cast<const void *>(this));
  this->conn_ = bh_->onSuccess(&CbSequence::onSubNodeSuccess, this);
  this->conn2_ = bh_->onFailure(&CbSequence::onSubNodeAbort, this);

  RCLCPP_INFO(
    getLogger(), "[SequenceNode %p] subnode %s on entry", static_cast<const void *>(this),
    currentNodeName.c_str());
  bh_->executeOnEntry();
  lock.unlock();
  RCLCPP_INFO(
    getLogger(), "[SequenceNode %p] subnode %s on entry finished", static_cast<const void *>(this),
    currentNodeName.c_str());
  bh_->waitOnEntryThread(false);  // we do not request to finish to keep on subscriptions
  RCLCPP_INFO(
    getLogger(), "[SequenceNode %p] subnode %s thread finished", static_cast<const void *>(this),
    currentNodeName.c_str());
  return true;
}

void CbSequence::onEntry()
{
  bool started = this->recursiveConsumeNext();
  while (started && !sequenceNodes_.empty())
  {
    bool is_shutdown_requested = this->isShutdownRequested();
    if (is_shutdown_requested)
    {
      break;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO_THROTTLE(
      getLogger(), *(getNode()->get_clock()), 1000,
      "[CbSequence %p] Waiting for subnodes to finish %ld. Current head Behavior: %s ",
      static_cast<const void *>(this), sequenceNodes_.size(), demangleType(&typeid(*bh_)).c_str());

    if (consume_)
    {
      this->conn_.disconnect();
      this->conn2_.disconnect();

      sequenceNodes_.pop_front();
      consume_--;

      if (sequenceNodes_.size() > 0)
      {
        started = this->recursiveConsumeNext();
      }
    }
  }

  if (sequenceNodes_.empty())
  {
    RCLCPP_INFO(
      getLogger(), "[CbSequence %p] All subnodes finished", static_cast<const void *>(this));
    this->postSuccessEvent();
  }
  else
  {
    RCLCPP_INFO(getLogger(), "[CbSequence %p] Aborting", static_cast<const void *>(this));
    this->postFailureEvent();
  }
}

void CbSequence::onSubNodeSuccess()
{
  RCLCPP_INFO(
    getLogger(), "[CbSequence %p] Success NextCbSequence %ld", static_cast<const void *>(this),
    sequenceNodes_.size());
  consume_++;
}

void CbSequence::onSubNodeAbort()
{
  RCLCPP_INFO(
    getLogger(), "[CbSequence %p] Abort NextCbSequence %ld", static_cast<const void *>(this),
    sequenceNodes_.size());
  this->conn_.disconnect();
  this->conn2_.disconnect();

  this->postFailureEvent();
  RCLCPP_INFO(
    getLogger(), "[CbSequence %p] Abort NextCbSequence requesting for force finish",
    static_cast<const void *>(this));
  this->requestForceFinish();
  consume_++;
}

}  // namespace client_behaviors

}  // namespace smacc2
