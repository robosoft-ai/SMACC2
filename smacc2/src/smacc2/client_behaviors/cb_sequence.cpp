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
CbSequence::CbSequence()
{
}

void CbSequence::recursiveConsumeNext()
{
  RCLCPP_INFO(getLogger(), "[SequenceNode] on entry: %ld", sequenceNodes_.size());

  auto first = sequenceNodes_.front();
  auto onDelayedConfigureFn = first;

  RCLCPP_INFO(getLogger(), "[SequenceNode]Behavior on delayed sequence configure");
  auto bh_ = onDelayedConfigureFn();
  std::string currentNodeName = bh_->getName();

  RCLCPP_INFO(getLogger(), "[SequenceNode]Subscribing OnSuccess");
  this->conn_ = bh_->onSuccess(&CbSequence::onSubNodeSuccess, this);
  this->conn_ = bh_->onSuccess(&CbSequence::onSubNodeAbort, this);

  RCLCPP_INFO(getLogger(), "[SequenceNode] subnode %s on entry", currentNodeName.c_str());
  bh_->executeOnEntry();
  RCLCPP_INFO(getLogger(), "[SequenceNode] subnode %s on entry finished", currentNodeName.c_str());
  bh_->waitOnEntryThread();
}

void CbSequence::onEntry()
{
  this->recursiveConsumeNext();
  while (!sequenceNodes_.empty() && !this->isShutdownRequested())
  {
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO_THROTTLE(getLogger(), *(getNode()->get_clock()), 1000, "[CbSequence] Waiting for subnodes to finish %ld",
                         sequenceNodes_.size());
  }

  if (sequenceNodes_.empty())
  {
    RCLCPP_INFO(getLogger(), "[CbSequence] All subnodes finished");
    this->postSuccessEvent();
  }
  else
  {
    RCLCPP_INFO(getLogger(), "[CbSequence] Aborting");
    this->postFailureEvent();
  }
}

void CbSequence::onSubNodeSuccess()
{
  RCLCPP_INFO(getLogger(), "[CbSequence] Success NextCbSequence %ld", sequenceNodes_.size());
  this->conn_.disconnect();
  sequenceNodes_.pop_front();
  if (sequenceNodes_.size() > 0)
  {
    this->recursiveConsumeNext();
  }
}

void CbSequence::onSubNodeAbort()
{
  this->postFailureEvent();
  this->requestForceFinish();
}

}  // namespace client_behaviors

}  // namespace smacc2
