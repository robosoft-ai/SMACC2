// Copyright 2024 Your Organization
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

#pragma once

#include <smacc2/smacc.hpp>
#include <smacc2/client_bases/smacc_action_client_base.hpp>
#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <atomic>

// Include your ROS2 message/action types here
// Example for action-based client:
// #include <your_msgs/action/your_action.hpp>
// Example for topic-based client:
// #include <your_msgs/msg/your_message.hpp>

namespace cl_template
{
// ============================================================================
// Event Definitions - Define custom events for your client
// ============================================================================
template <typename TSource, typename TOrthogonal>
struct EvTemplateSuccess : sc::event<EvTemplateSuccess<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvTemplateFailure : sc::event<EvTemplateFailure<TSource, TOrthogonal>>
{
};

template <typename TSource, typename TOrthogonal>
struct EvTemplateDataReceived : sc::event<EvTemplateDataReceived<TSource, TOrthogonal>>
{
};

// ============================================================================
// Option 1: Action-Based Client Template
// Inherit from SmaccActionClientBase for action server interaction
// ============================================================================
/*
template <typename TAction>
class ClTemplateAction : public smacc2::client_bases::SmaccActionClientBase<TAction>
{
public:
  using Base = smacc2::client_bases::SmaccActionClientBase<TAction>;
  using typename Base::GoalHandle;
  using typename Base::WrappedResult;
  using typename Base::ResultCallback;
  
  // Signal for custom result handling
  typedef smacc2::SmaccSignal<void(const WrappedResult &)> TemplateResultSignal;
  
  // Constructor with configurable action name
  ClTemplateAction(std::string actionName = "/template_action");
  
  // Destructor
  virtual ~ClTemplateAction();
  
  // Optional: Override onInitialize for custom initialization
  virtual void onInitialize() override;
  
  // Custom methods for your client
  void sendCustomGoal();
  void cancelCurrentGoal();
  
private:
  // Private members for internal state
  std::string action_name_;
  // Add more members as needed
};
*/

// ============================================================================
// Option 2: Topic-Based Client Template  
// Inherit from ISmaccClient for general clients or SmaccSubscriberClient
// ============================================================================
class ClTemplate : public smacc2::ISmaccClient
{
public:
  // Constructor
  ClTemplate(std::string topicName = "/template_topic");
  
  // Destructor
  virtual ~ClTemplate();
  
  // Lifecycle methods
  virtual void onInitialize() override;
  
  // Custom public methods
  void startOperation();
  void stopOperation();
  void configureParameters();
  
  // Public signals for event communication
  smacc2::SmaccSignal<void()> onDataReceived_;
  smacc2::SmaccSignal<void(int)> onStatusChanged_;
  
protected:
  // ROS2 communication members
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Configuration parameters
  std::string topic_name_;
  double publish_rate_;
  bool is_active_;
  
  // Callback methods
  void messageCallback(const std_msgs::msg::String::SharedPtr msg);
  void timerCallback();
  
private:
  // Internal state management
  void updateInternalState();
  bool validateConfiguration();
  
  // Private members
  std::mutex state_mutex_;
  std::atomic<bool> processing_;
};

// ============================================================================
// Option 3: Service-Based Client Template
// ============================================================================
/*
template <typename TService>
class ClTemplateService : public smacc2::ISmaccClient
{
public:
  ClTemplateService(std::string serviceName = "/template_service");
  virtual ~ClTemplateService();
  
  virtual void onInitialize() override;
  
  // Synchronous service call
  bool callService(
    const typename TService::Request::SharedPtr request,
    typename TService::Response::SharedPtr response);
  
  // Asynchronous service call
  void callServiceAsync(
    const typename TService::Request::SharedPtr request,
    std::function<void(typename TService::Response::SharedPtr)> callback);
  
private:
  typename rclcpp::Client<TService>::SharedPtr service_client_;
  std::string service_name_;
};
*/

}  // namespace cl_template