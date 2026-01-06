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
#include <cl_lifecycle_node/cl_lifecycle_node.hpp>
#include <cl_lifecycle_node/components/cp_lifecycle_event_monitor.hpp>
#include <string>

namespace cl_lifecycle_node
{
ClLifecycleNode::ClLifecycleNode(std::string node_name)
: nodeName_(node_name), eventMonitor_(nullptr)
{
}

ClLifecycleNode::~ClLifecycleNode() {}

void ClLifecycleNode::onInitialize()
{
  // Create service clients
  client_get_state_ = this->getNode()->create_client<lifecycle_msgs::srv::GetState>(
    this->nodeName_ + node_get_state_topic);
  client_change_state_ = this->getNode()->create_client<lifecycle_msgs::srv::ChangeState>(
    this->nodeName_ + node_change_state_topic);
}

void ClLifecycleNode::changeState(uint8_t state)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = state;
  auto future = client_change_state_->async_send_request(request);
  future.wait();
}

void ClLifecycleNode::activate()
{
  changeState(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

void ClLifecycleNode::deactivate()
{
  changeState(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

void ClLifecycleNode::configure()
{
  changeState(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

void ClLifecycleNode::cleanup()
{
  changeState(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
}

void ClLifecycleNode::shutdown()
{
  changeState(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
}

}  // namespace cl_lifecycle_node
