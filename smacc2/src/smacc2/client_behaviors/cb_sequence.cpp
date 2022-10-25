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

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/client_behaviors/cb_sequence.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace smacc2
{
namespace client_behaviors
{
CbSequence::CbSequence() {}

void CbSequence::recursiveConsumeNext()
{
  RCLCPP_INFO(
    getLogger(), "[SequenceNode %ld] next onEntry: %ld", (long)this, sequenceNodes_.size());

  auto first = sequenceNodes_.front();
  auto onDelayedConfigureFn = first;

  RCLCPP_INFO(getLogger(), "[SequenceNode %ld] Behavior on delayed sequence configure", (long)this);
  bh_ = onDelayedConfigureFn();
  std::string currentNodeName = bh_->getName();

  RCLCPP_INFO(getLogger(), "[SequenceNode %ld] Subscribing OnSuccess", (long)this);
  this->conn_ = bh_->onSuccess(&CbSequence::onSubNodeSuccess, this);
  this->conn2_ = bh_->onFailure(&CbSequence::onSubNodeAbort, this);

  RCLCPP_INFO(
    getLogger(), "[SequenceNode %ld] subnode %s on entry", (long)this, currentNodeName.c_str());
  bh_->executeOnEntry();
  RCLCPP_INFO(
    getLogger(), "[SequenceNode %ld] subnode %s on entry finished", (long)this,
    currentNodeName.c_str());
  bh_->waitOnEntryThread(false);  // we do not request to finish to keep on subscriptions
  RCLCPP_INFO(
    getLogger(), "[SequenceNode %ld] subnode %s thread finished", (long)this,
    currentNodeName.c_str());
}

void CbSequence::onEntry()
{
  this->recursiveConsumeNext();
  while (!sequenceNodes_.empty())
  {
    bool is_shutdown_requested = this->isShutdownRequested();
    if (is_shutdown_requested)
    {
      break;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO_THROTTLE(
      getLogger(), *(getNode()->get_clock()), 1000,
      "[CbSequence %ld] Waiting for subnodes to finish %ld. Current head Behavior: %s ", (long)this,
      sequenceNodes_.size(), demangleType(&typeid(*bh_)).c_str());

    if (consume_)
    {
      // bh_->waitOnEntryThread();
      this->conn_.disconnect();
      this->conn2_.disconnect();

      sequenceNodes_.pop_front();
      consume_--;

      if (sequenceNodes_.size() > 0)
      {
        this->recursiveConsumeNext();
      }
    }
  }

  if (sequenceNodes_.empty())
  {
    RCLCPP_INFO(getLogger(), "[CbSequence %ld] All subnodes finished", (long)this);
    this->postSuccessEvent();
  }
  else
  {
    RCLCPP_INFO(getLogger(), "[CbSequence %ld] Aborting", (long)this);
    this->postFailureEvent();
  }
}

void CbSequence::onSubNodeSuccess()
{
  RCLCPP_INFO(
    getLogger(), "[CbSequence %ld] Success NextCbSequence %ld", (long)this, sequenceNodes_.size());
  consume_++;
}

void CbSequence::onSubNodeAbort()
{
  RCLCPP_INFO(
    getLogger(), "[CbSequence %ld] Abort NextCbSequence %ld", (long)this, sequenceNodes_.size());
  // bh_->waitOnEntryThread();
  this->conn_.disconnect();
  this->conn2_.disconnect();

  this->postFailureEvent();
  RCLCPP_INFO(
    getLogger(), "[CbSequence %ld] Abort NextCbSequence requesting for force finish", (long)this);
  this->requestForceFinish();
  consume_++;
}

}  // namespace client_behaviors

}  // namespace smacc2
