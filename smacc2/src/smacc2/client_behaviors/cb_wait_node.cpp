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

#include <smacc2/client_behaviors/cb_wait_node.hpp>

namespace smacc2
{
namespace client_behaviors
{
CbWaitNode::CbWaitNode(std::string nodeName) : nodeName_(nodeName), rate_(5) {}

void CbWaitNode::onEntry()
{
  bool found = false;
  while (!this->isShutdownRequested() && !found)
  {
    std::stringstream ss;
    auto nodenames = getNode()->get_node_names();
    for (auto n : nodenames)
    {
      ss << " - " << n << std::endl;

      if (n == nodeName_) found = true;
    }

    auto totalstr = ss.str();
    RCLCPP_INFO_STREAM(
      getLogger(), "[" << getName() << "] on entry, listing nodes (" << nodenames.size() << ")"
                       << std::endl
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
