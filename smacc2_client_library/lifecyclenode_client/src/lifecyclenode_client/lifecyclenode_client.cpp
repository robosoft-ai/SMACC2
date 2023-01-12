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
#include <lifecyclenode_client/lifecyclenode_client.hpp>
#include <string>

namespace cl_lifecyclenode
{
ClLifecycleNode::ClLifecycleNode(std::string node_name) : nodeName_(node_name) {}

ClLifecycleNode::~ClLifecycleNode() {}

void ClLifecycleNode::onInitialize()
{
  client_get_state_ = this->getNode()->create_client<lifecycle_msgs::srv::GetState>(
    this->nodeName_ + "/" + node_get_state_topic);
  client_change_state_ = this->getNode()->create_client<lifecycle_msgs::srv::ChangeState>(
    this->nodeName_ + "/" + node_change_state_topic);

  this->subscription_transition_event_ =
    this->getNode() /*  */->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      this->nodeName_ + "/" + node_transition_event_topic, 10,
      std::bind(&ClLifecycleNode::onTransitionEvent, this, std::placeholders::_1));
}

void ClLifecycleNode::changeState(uint8_t state)
{
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = state;
  auto future = client_change_state_->async_send_request(request);
  future.wait();
}

void ClLifecycleNode::onTransitionEvent(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
{
  switch (msg->transition.id)
  {
    case lifecycle_msgs::msg::Transition::TRANSITION_CREATE:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_DESTROY:
      break;

    case lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_SUCCESS:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_FAILURE:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_CONFIGURE_ERROR:
      break;

    case lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_SUCCESS:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_FAILURE:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_CLEANUP_ERROR:
      break;

    case lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_SUCCESS:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_FAILURE:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_ERROR:
      break;

    case lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_SUCCESS:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_FAILURE:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_DEACTIVATE_ERROR:
      break;

    case lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_SUCCESS:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_FAILURE:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_SHUTDOWN_ERROR:
      break;

    case lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_SUCCESS:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_FAILURE:
      break;
    case lifecycle_msgs::msg::Transition::TRANSITION_ON_ERROR_ERROR:
      break;

    default:
      break;
  }

}  // namespace cl_lifecyclenode
}  // namespace cl_lifecyclenode
