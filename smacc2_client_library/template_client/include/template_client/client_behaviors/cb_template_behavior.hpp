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
#include <template_client/cl_template.hpp>
#include <optional>

namespace cl_template
{
// ============================================================================
// Behavior Configuration Options
// ============================================================================
struct CbTemplateBehaviorOptions
{
  // Example configuration parameters
  std::optional<double> timeout_seconds;
  std::optional<int> max_retries;
  std::optional<std::string> custom_parameter;
  std::optional<bool> enable_logging;
  
  // Default constructor
  CbTemplateBehaviorOptions() 
  : timeout_seconds(10.0),
    max_retries(3),
    enable_logging(true)
  {}
};

// ============================================================================
// Basic Template Behavior
// ============================================================================
class CbTemplateBehavior : public smacc2::SmaccClientBehavior
{
public:
  // Configuration options
  CbTemplateBehaviorOptions options;
  
  // Constructors
  CbTemplateBehavior();
  explicit CbTemplateBehavior(const CbTemplateBehaviorOptions& opts);
  
  // Destructor
  virtual ~CbTemplateBehavior();
  
  // Lifecycle methods (required)
  virtual void onEntry() override;
  virtual void onExit() override;
  
  // Optional: Called when state machine allocates this behavior to an orthogonal
  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    // Setup event posting for this orthogonal
    postSuccessEvent_ = [this]() {
      this->template postEvent<EvTemplateSuccess<TSourceObject, TOrthogonal>>();
    };
    
    postFailureEvent_ = [this]() {
      this->template postEvent<EvTemplateFailure<TSourceObject, TOrthogonal>>();
    };
  }
  
  // Public methods for behavior control
  void configure(const CbTemplateBehaviorOptions& opts);
  void executeCustomAction();
  void cancelCurrentOperation();
  
protected:
  // Client reference
  ClTemplate* templateClient_;
  
  // Event posting functions
  std::function<void()> postSuccessEvent_;
  std::function<void()> postFailureEvent_;
  
  // Internal state
  bool is_active_;
  int retry_count_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  
  // Signal connections
  boost::signals2::scoped_connection data_received_connection_;
  boost::signals2::scoped_connection status_changed_connection_;
  
  // Protected methods
  virtual void onTimeout();
  virtual void onDataReceived();
  virtual void onStatusChanged(int status);
  virtual bool shouldRetry();
  
private:
  // Private implementation details
  void setupConnections();
  void teardownConnections();
  void startTimeoutTimer();
  void stopTimeoutTimer();
};

// ============================================================================
// Async Template Behavior - Example of async operations
// ============================================================================
class CbTemplateAsyncBehavior : public CbTemplateBehavior
{
public:
  CbTemplateAsyncBehavior();
  
  // Override lifecycle methods for async behavior
  virtual void onEntry() override;
  virtual void onExit() override;
  
  // Async operation methods
  void startAsyncOperation();
  void onAsyncOperationComplete(bool success);
  
private:
  std::thread async_thread_;
  std::atomic<bool> cancel_requested_;
  std::mutex async_mutex_;
  std::condition_variable async_cv_;
};

// ============================================================================
// Periodic Template Behavior - Example of periodic operations
// ============================================================================
class CbTemplatePeriodicBehavior : public smacc2::SmaccClientBehavior
{
public:
  // Constructor with period configuration
  CbTemplatePeriodicBehavior(double period_seconds = 1.0);
  virtual ~CbTemplatePeriodicBehavior();
  
  // Lifecycle methods
  virtual void onEntry() override;
  virtual void onExit() override;
  
  // Configure for event posting
  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    postPeriodicEvent_ = [this]() {
      this->template postEvent<EvTemplateDataReceived<TSourceObject, TOrthogonal>>();
    };
  }
  
  // Public control methods
  void setPeriod(double period_seconds);
  void pause();
  void resume();
  
protected:
  // Periodic callback
  virtual void onPeriodicCallback();
  
private:
  ClTemplate* templateClient_;
  rclcpp::TimerBase::SharedPtr periodic_timer_;
  double period_seconds_;
  bool is_paused_;
  int iteration_count_;
  
  std::function<void()> postPeriodicEvent_;
};

// ============================================================================
// Conditional Template Behavior - Example with conditions
// ============================================================================
class CbTemplateConditionalBehavior : public smacc2::SmaccClientBehavior
{
public:
  // Condition function type
  using ConditionFunction = std::function<bool()>;
  
  // Constructor with condition
  CbTemplateConditionalBehavior(ConditionFunction condition);
  virtual ~CbTemplateConditionalBehavior();
  
  // Lifecycle methods
  virtual void onEntry() override;
  virtual void onExit() override;
  
  // Set/update condition
  void setCondition(ConditionFunction condition);
  
  // Configure for event posting
  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    postConditionMetEvent_ = [this]() {
      this->template postEvent<EvTemplateSuccess<TSourceObject, TOrthogonal>>();
    };
    
    postConditionFailedEvent_ = [this]() {
      this->template postEvent<EvTemplateFailure<TSourceObject, TOrthogonal>>();
    };
  }
  
protected:
  // Check condition periodically
  void checkCondition();
  
private:
  ClTemplate* templateClient_;
  ConditionFunction condition_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  
  std::function<void()> postConditionMetEvent_;
  std::function<void()> postConditionFailedEvent_;
  
  bool condition_met_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::seconds timeout_;
};

}  // namespace cl_template