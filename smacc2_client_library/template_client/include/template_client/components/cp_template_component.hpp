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
#include <smacc2/component.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <deque>
#include <optional>

namespace cl_template
{
// ============================================================================
// Data Storage Component - Example of a component that stores data
// ============================================================================
class CpTemplateDataStorage : public smacc2::ISmaccComponent
{
public:
  // Constructor
  CpTemplateDataStorage(size_t max_buffer_size = 100);
  
  // Destructor
  virtual ~CpTemplateDataStorage();
  
  // Initialize component
  virtual void onInitialize() override;
  
  // Data management methods
  void storeData(const std::string& key, const std::string& value);
  std::optional<std::string> getData(const std::string& key) const;
  void clearData();
  void clearData(const std::string& key);
  
  // Buffer management for time-series data
  template<typename T>
  void pushToBuffer(const T& data)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (data_buffer_.size() >= max_buffer_size_)
    {
      data_buffer_.pop_front();
    }
    
    // Store as string representation (simplified)
    std::stringstream ss;
    ss << data;
    data_buffer_.push_back(ss.str());
  }
  
  // Get buffer contents
  std::vector<std::string> getBufferContents() const;
  size_t getBufferSize() const;
  void clearBuffer();
  
  // Statistics
  size_t getDataCount() const;
  std::vector<std::string> getKeys() const;
  
protected:
  // Data storage
  mutable std::mutex data_mutex_;
  std::map<std::string, std::string> data_storage_;
  
  // Buffer for time-series data
  mutable std::mutex buffer_mutex_;
  std::deque<std::string> data_buffer_;
  size_t max_buffer_size_;
};

// ============================================================================
// State Tracker Component - Example of a component that tracks state
// ============================================================================
class CpTemplateStateTracker : public smacc2::ISmaccComponent
{
public:
  // State enum
  enum class State
  {
    IDLE,
    INITIALIZING,
    RUNNING,
    PAUSED,
    ERROR,
    COMPLETED
  };
  
  // State transition callback type
  using StateChangeCallback = std::function<void(State, State)>;
  
  // Constructor
  CpTemplateStateTracker();
  
  // Destructor
  virtual ~CpTemplateStateTracker();
  
  // Initialize component
  virtual void onInitialize() override;
  
  // State management
  void setState(State new_state);
  State getState() const;
  std::string getStateString() const;
  
  // State history
  std::vector<std::pair<State, rclcpp::Time>> getStateHistory() const;
  void clearHistory();
  
  // Register callback for state changes
  void registerStateChangeCallback(StateChangeCallback callback);
  
  // Utility methods
  bool isInState(State state) const;
  bool hasBeenInState(State state) const;
  rclcpp::Duration getTimeInCurrentState() const;
  
protected:
  // Convert state to string
  std::string stateToString(State state) const;
  
private:
  // Current state
  mutable std::mutex state_mutex_;
  State current_state_;
  rclcpp::Time state_entry_time_;
  
  // State history
  std::vector<std::pair<State, rclcpp::Time>> state_history_;
  
  // Callbacks
  std::vector<StateChangeCallback> state_change_callbacks_;
  
  // Node for time access
  rclcpp::Node::SharedPtr node_;
};

// ============================================================================
// Transform Manager Component - Example of a component that manages transforms
// ============================================================================
class CpTemplateTransformManager : public smacc2::ISmaccComponent
{
public:
  // Constructor
  CpTemplateTransformManager(
    const std::string& reference_frame = "map",
    const std::string& target_frame = "base_link");
  
  // Destructor
  virtual ~CpTemplateTransformManager();
  
  // Initialize component
  virtual void onInitialize() override;
  
  // Frame management
  void setReferenceFrame(const std::string& frame);
  void setTargetFrame(const std::string& frame);
  std::string getReferenceFrame() const;
  std::string getTargetFrame() const;
  
  // Pose tracking
  void updateCurrentPose(const geometry_msgs::msg::PoseStamped& pose);
  std::optional<geometry_msgs::msg::PoseStamped> getCurrentPose() const;
  
  // Pose history
  void recordPose(const geometry_msgs::msg::PoseStamped& pose);
  std::vector<geometry_msgs::msg::PoseStamped> getPoseHistory() const;
  void clearPoseHistory();
  
  // Utility methods
  double getDistanceFromLastPose() const;
  double getTotalDistance() const;
  geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped& pose,
    const std::string& target_frame) const;
  
protected:
  // Calculate distance between two poses
  double calculateDistance(
    const geometry_msgs::msg::PoseStamped& pose1,
    const geometry_msgs::msg::PoseStamped& pose2) const;
  
private:
  // Frame names
  std::string reference_frame_;
  std::string target_frame_;
  
  // Current pose
  mutable std::mutex pose_mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> current_pose_;
  
  // Pose history
  std::vector<geometry_msgs::msg::PoseStamped> pose_history_;
  size_t max_history_size_;
  
  // Distance tracking
  double total_distance_;
};

// ============================================================================
// Configuration Manager Component - Example of a component that manages config
// ============================================================================
class CpTemplateConfigManager : public smacc2::ISmaccComponent
{
public:
  // Configuration structure
  struct Config
  {
    double param1;
    int param2;
    std::string param3;
    bool param4;
    
    // Default values
    Config() : param1(1.0), param2(10), param3("default"), param4(true) {}
  };
  
  // Constructor
  CpTemplateConfigManager();
  
  // Destructor
  virtual ~CpTemplateConfigManager();
  
  // Initialize component
  virtual void onInitialize() override;
  
  // Configuration management
  void setConfig(const Config& config);
  Config getConfig() const;
  
  // Individual parameter access
  void setParameter(const std::string& name, const rclcpp::ParameterValue& value);
  rclcpp::ParameterValue getParameter(const std::string& name) const;
  
  // Load/Save configuration
  bool loadFromYaml(const std::string& filepath);
  bool saveToYaml(const std::string& filepath) const;
  
  // Parameter validation
  bool validateConfig(const Config& config) const;
  
  // Reset to defaults
  void resetToDefaults();
  
  // Configuration change notification
  smacc2::SmaccSignal<void(const Config&)> onConfigChanged_;
  
protected:
  // Update ROS parameters
  void updateROSParameters();
  
  // Load parameters from ROS
  void loadROSParameters();
  
private:
  // Current configuration
  mutable std::mutex config_mutex_;
  Config current_config_;
  Config default_config_;
  
  // ROS parameter handling
  rclcpp::Node::SharedPtr node_;
  std::vector<rclcpp::Parameter> parameters_;
};

}  // namespace cl_template