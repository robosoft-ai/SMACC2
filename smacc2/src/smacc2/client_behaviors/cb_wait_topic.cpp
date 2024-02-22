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

#include <smacc2/client_behaviors/cb_wait_topic.hpp>

namespace smacc2
{
namespace client_behaviors
{
CbWaitTopic::CbWaitTopic(std::string nodeName) : topicName_(nodeName), rate_(5) {}

CbWaitTopic::~CbWaitTopic() {}

void CbWaitTopic::onEntry()
{
  bool found = false;
  while (!this->isShutdownRequested() && !found)
  {
    RCLCPP_INFO_STREAM_THROTTLE(
      getLogger(), *(getNode()->get_clock()), 1000,
      "[" << getName() << "] waiting topic: " << topicName_);
    std::stringstream ss;
    auto topicnames = getNode()->get_topic_names_and_types();

    for (auto & t : topicnames)
    {
      ss << t.first << std::endl;
      if (t.first == topicName_)
      {
        found = true;
      }
    }

    auto totalstr = ss.str();
    RCLCPP_INFO_STREAM_THROTTLE(
      getLogger(), *(getNode()->get_clock()), 5000,
      "[" << getName() << "] still waiting topic " << topicName_ << ", listing topics ("
          << topicnames.size() << ")" << std::endl
          << totalstr);

    rate_.sleep();
  }

  if (found)
  {
    this->postSuccessEvent();
  }
  else
  {
    this->postFailureEvent();
  }
}

}  // namespace client_behaviors
}  // namespace smacc2
