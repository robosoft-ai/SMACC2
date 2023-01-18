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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/string.hpp>

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

class LifecycleExampleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  LifecycleExampleNode() : rclcpp_lifecycle::LifecycleNode("lifecycle_example_node")
  {
    // Create a publisher for the "example_topic" topic
    publisher_ = this->create_publisher<std_msgs::msg::String>("example_topic", rclcpp::QoS(10));
  }

  LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Configuring node, previous state: '%s'", previous_state.label().c_str());
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Activating node, previous state: '%s'", previous_state.label().c_str());
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating node, previous state: '%s'", previous_state.label().c_str());
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up node, previous state: '%s'", previous_state.label().c_str());
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down node, previous state: '%s'", previous_state.label().c_str());
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state)
  {
    RCLCPP_ERROR(this->get_logger(), "Error occurred, previous state: '%s'", previous_state.label().c_str());
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
};


int main()
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<LifecycleExampleNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}
