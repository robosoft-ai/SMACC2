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

#include <template_client/components/cp_template_component.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <cmath>

namespace cl_template
{
// ============================================================================
// CpTemplateDataStorage Implementation
// ============================================================================
CpTemplateDataStorage::CpTemplateDataStorage(size_t max_buffer_size)
: max_buffer_size_(max_buffer_size)
{
  RCLCPP_DEBUG(getLogger(), "[CpTemplateDataStorage] Constructor");
}

CpTemplateDataStorage::~CpTemplateDataStorage()
{
  RCLCPP_DEBUG(getLogger(), "[CpTemplateDataStorage] Destructor");
}

void CpTemplateDataStorage::onInitialize()
{
  RCLCPP_INFO(getLogger(), "[CpTemplateDataStorage] Initializing component");
}

void CpTemplateDataStorage::storeData(const std::string& key, const std::string& value)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_storage_[key] = value;
  RCLCPP_DEBUG(getLogger(), "[CpTemplateDataStorage] Stored data: %s = %s", 
               key.c_str(), value.c_str());
}

std::optional<std::string> CpTemplateDataStorage::getData(const std::string& key) const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  auto it = data_storage_.find(key);
  if (it != data_storage_.end())
  {
    return it->second;
  }
  return std::nullopt;
}

void CpTemplateDataStorage::clearData()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_storage_.clear();
  RCLCPP_INFO(getLogger(), "[CpTemplateDataStorage] Cleared all data");
}

void CpTemplateDataStorage::clearData(const std::string& key)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  data_storage_.erase(key);
  RCLCPP_DEBUG(getLogger(), "[CpTemplateDataStorage] Cleared data for key: %s", key.c_str());
}

std::vector<std::string> CpTemplateDataStorage::getBufferContents() const
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  return std::vector<std::string>(data_buffer_.begin(), data_buffer_.end());
}

size_t CpTemplateDataStorage::getBufferSize() const
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  return data_buffer_.size();
}

void CpTemplateDataStorage::clearBuffer()
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  data_buffer_.clear();
  RCLCPP_INFO(getLogger(), "[CpTemplateDataStorage] Cleared buffer");
}

size_t CpTemplateDataStorage::getDataCount() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return data_storage_.size();
}

std::vector<std::string> CpTemplateDataStorage::getKeys() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  std::vector<std::string> keys;
  for (const auto& pair : data_storage_)
  {
    keys.push_back(pair.first);
  }
  return keys;
}

// ============================================================================
// CpTemplateStateTracker Implementation
// ============================================================================
CpTemplateStateTracker::CpTemplateStateTracker()
: current_state_(State::IDLE)
{
  RCLCPP_DEBUG(getLogger(), "[CpTemplateStateTracker] Constructor");
}

CpTemplateStateTracker::~CpTemplateStateTracker()
{
  RCLCPP_DEBUG(getLogger(), "[CpTemplateStateTracker] Destructor");
}

void CpTemplateStateTracker::onInitialize()
{
  RCLCPP_INFO(getLogger(), "[CpTemplateStateTracker] Initializing component");
  node_ = getNode();
  state_entry_time_ = node_->now();
  
  // Record initial state
  state_history_.emplace_back(current_state_, state_entry_time_);
}

void CpTemplateStateTracker::setState(State new_state)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  
  if (new_state == current_state_)
  {
    RCLCPP_DEBUG(getLogger(), "[CpTemplateStateTracker] State unchanged: %s", 
                 stateToString(new_state).c_str());
    return;
  }
  
  State old_state = current_state_;
  current_state_ = new_state;
  state_entry_time_ = node_->now();
  
  // Record in history
  state_history_.emplace_back(new_state, state_entry_time_);
  
  RCLCPP_INFO(getLogger(), "[CpTemplateStateTracker] State transition: %s -> %s",
              stateToString(old_state).c_str(), stateToString(new_state).c_str());
  
  // Notify callbacks
  for (const auto& callback : state_change_callbacks_)
  {
    callback(old_state, new_state);
  }
}

CpTemplateStateTracker::State CpTemplateStateTracker::getState() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_state_;
}

std::string CpTemplateStateTracker::getStateString() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return stateToString(current_state_);
}

std::vector<std::pair<CpTemplateStateTracker::State, rclcpp::Time>> 
CpTemplateStateTracker::getStateHistory() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_history_;
}

void CpTemplateStateTracker::clearHistory()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_history_.clear();
  // Keep current state in history
  state_history_.emplace_back(current_state_, state_entry_time_);
}

void CpTemplateStateTracker::registerStateChangeCallback(StateChangeCallback callback)
{
  state_change_callbacks_.push_back(callback);
}

bool CpTemplateStateTracker::isInState(State state) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_state_ == state;
}

bool CpTemplateStateTracker::hasBeenInState(State state) const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  for (const auto& [hist_state, time] : state_history_)
  {
    if (hist_state == state)
      return true;
  }
  return false;
}

rclcpp::Duration CpTemplateStateTracker::getTimeInCurrentState() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return node_->now() - state_entry_time_;
}

std::string CpTemplateStateTracker::stateToString(State state) const
{
  switch (state)
  {
    case State::IDLE: return "IDLE";
    case State::INITIALIZING: return "INITIALIZING";
    case State::RUNNING: return "RUNNING";
    case State::PAUSED: return "PAUSED";
    case State::ERROR: return "ERROR";
    case State::COMPLETED: return "COMPLETED";
    default: return "UNKNOWN";
  }
}

// ============================================================================
// CpTemplateTransformManager Implementation
// ============================================================================
CpTemplateTransformManager::CpTemplateTransformManager(
  const std::string& reference_frame,
  const std::string& target_frame)
: reference_frame_(reference_frame),
  target_frame_(target_frame),
  max_history_size_(1000),
  total_distance_(0.0)
{
  RCLCPP_DEBUG(getLogger(), "[CpTemplateTransformManager] Constructor");
}

CpTemplateTransformManager::~CpTemplateTransformManager()
{
  RCLCPP_DEBUG(getLogger(), "[CpTemplateTransformManager] Destructor");
}

void CpTemplateTransformManager::onInitialize()
{
  RCLCPP_INFO(getLogger(), "[CpTemplateTransformManager] Initializing with frames: %s -> %s",
              reference_frame_.c_str(), target_frame_.c_str());
}

void CpTemplateTransformManager::setReferenceFrame(const std::string& frame)
{
  reference_frame_ = frame;
  RCLCPP_INFO(getLogger(), "[CpTemplateTransformManager] Reference frame set to: %s", frame.c_str());
}

void CpTemplateTransformManager::setTargetFrame(const std::string& frame)
{
  target_frame_ = frame;
  RCLCPP_INFO(getLogger(), "[CpTemplateTransformManager] Target frame set to: %s", frame.c_str());
}

std::string CpTemplateTransformManager::getReferenceFrame() const
{
  return reference_frame_;
}

std::string CpTemplateTransformManager::getTargetFrame() const
{
  return target_frame_;
}

void CpTemplateTransformManager::updateCurrentPose(const geometry_msgs::msg::PoseStamped& pose)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  
  // Calculate distance from last pose if exists
  if (current_pose_.has_value())
  {
    double distance = calculateDistance(current_pose_.value(), pose);
    total_distance_ += distance;
  }
  
  current_pose_ = pose;
  RCLCPP_DEBUG(getLogger(), "[CpTemplateTransformManager] Updated current pose");
}

std::optional<geometry_msgs::msg::PoseStamped> CpTemplateTransformManager::getCurrentPose() const
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return current_pose_;
}

void CpTemplateTransformManager::recordPose(const geometry_msgs::msg::PoseStamped& pose)
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  
  if (pose_history_.size() >= max_history_size_)
  {
    pose_history_.erase(pose_history_.begin());
  }
  
  pose_history_.push_back(pose);
  RCLCPP_DEBUG(getLogger(), "[CpTemplateTransformManager] Recorded pose, history size: %zu", 
               pose_history_.size());
}

std::vector<geometry_msgs::msg::PoseStamped> CpTemplateTransformManager::getPoseHistory() const
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return pose_history_;
}

void CpTemplateTransformManager::clearPoseHistory()
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  pose_history_.clear();
  total_distance_ = 0.0;
  RCLCPP_INFO(getLogger(), "[CpTemplateTransformManager] Cleared pose history");
}

double CpTemplateTransformManager::getDistanceFromLastPose() const
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  
  if (!current_pose_.has_value() || pose_history_.empty())
  {
    return 0.0;
  }
  
  return calculateDistance(pose_history_.back(), current_pose_.value());
}

double CpTemplateTransformManager::getTotalDistance() const
{
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return total_distance_;
}

geometry_msgs::msg::PoseStamped CpTemplateTransformManager::transformPose(
  const geometry_msgs::msg::PoseStamped& pose,
  const std::string& target_frame) const
{
  // Note: This is a simplified version. In practice, you would use tf2 for proper transformation
  RCLCPP_WARN(getLogger(), "[CpTemplateTransformManager] Transform not implemented - returning original pose");
  return pose;
}

double CpTemplateTransformManager::calculateDistance(
  const geometry_msgs::msg::PoseStamped& pose1,
  const geometry_msgs::msg::PoseStamped& pose2) const
{
  double dx = pose2.pose.position.x - pose1.pose.position.x;
  double dy = pose2.pose.position.y - pose1.pose.position.y;
  double dz = pose2.pose.position.z - pose1.pose.position.z;
  
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// ============================================================================
// CpTemplateConfigManager Implementation
// ============================================================================
CpTemplateConfigManager::CpTemplateConfigManager()
{
  RCLCPP_DEBUG(getLogger(), "[CpTemplateConfigManager] Constructor");
  current_config_ = default_config_;
}

CpTemplateConfigManager::~CpTemplateConfigManager()
{
  RCLCPP_DEBUG(getLogger(), "[CpTemplateConfigManager] Destructor");
}

void CpTemplateConfigManager::onInitialize()
{
  RCLCPP_INFO(getLogger(), "[CpTemplateConfigManager] Initializing component");
  node_ = getNode();
  
  // Load parameters from ROS
  loadROSParameters();
}

void CpTemplateConfigManager::setConfig(const Config& config)
{
  if (!validateConfig(config))
  {
    RCLCPP_ERROR(getLogger(), "[CpTemplateConfigManager] Invalid configuration");
    return;
  }
  
  std::lock_guard<std::mutex> lock(config_mutex_);
  current_config_ = config;
  
  // Update ROS parameters
  updateROSParameters();
  
  // Notify listeners
  onConfigChanged_(current_config_);
  
  RCLCPP_INFO(getLogger(), "[CpTemplateConfigManager] Configuration updated");
}

CpTemplateConfigManager::Config CpTemplateConfigManager::getConfig() const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  return current_config_;
}

void CpTemplateConfigManager::setParameter(const std::string& name, const rclcpp::ParameterValue& value)
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  
  if (name == "param1")
  {
    current_config_.param1 = value.get<double>();
  }
  else if (name == "param2")
  {
    current_config_.param2 = value.get<int>();
  }
  else if (name == "param3")
  {
    current_config_.param3 = value.get<std::string>();
  }
  else if (name == "param4")
  {
    current_config_.param4 = value.get<bool>();
  }
  
  RCLCPP_DEBUG(getLogger(), "[CpTemplateConfigManager] Set parameter: %s", name.c_str());
}

rclcpp::ParameterValue CpTemplateConfigManager::getParameter(const std::string& name) const
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  
  if (name == "param1")
  {
    return rclcpp::ParameterValue(current_config_.param1);
  }
  else if (name == "param2")
  {
    return rclcpp::ParameterValue(current_config_.param2);
  }
  else if (name == "param3")
  {
    return rclcpp::ParameterValue(current_config_.param3);
  }
  else if (name == "param4")
  {
    return rclcpp::ParameterValue(current_config_.param4);
  }
  
  return rclcpp::ParameterValue();
}

bool CpTemplateConfigManager::loadFromYaml(const std::string& filepath)
{
  try
  {
    YAML::Node yaml = YAML::LoadFile(filepath);
    
    Config config;
    if (yaml["param1"]) config.param1 = yaml["param1"].as<double>();
    if (yaml["param2"]) config.param2 = yaml["param2"].as<int>();
    if (yaml["param3"]) config.param3 = yaml["param3"].as<std::string>();
    if (yaml["param4"]) config.param4 = yaml["param4"].as<bool>();
    
    setConfig(config);
    
    RCLCPP_INFO(getLogger(), "[CpTemplateConfigManager] Loaded configuration from: %s", 
                filepath.c_str());
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(getLogger(), "[CpTemplateConfigManager] Failed to load YAML: %s", e.what());
    return false;
  }
}

bool CpTemplateConfigManager::saveToYaml(const std::string& filepath) const
{
  try
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    YAML::Node yaml;
    yaml["param1"] = current_config_.param1;
    yaml["param2"] = current_config_.param2;
    yaml["param3"] = current_config_.param3;
    yaml["param4"] = current_config_.param4;
    
    std::ofstream fout(filepath);
    fout << yaml;
    fout.close();
    
    RCLCPP_INFO(getLogger(), "[CpTemplateConfigManager] Saved configuration to: %s", 
                filepath.c_str());
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(getLogger(), "[CpTemplateConfigManager] Failed to save YAML: %s", e.what());
    return false;
  }
}

bool CpTemplateConfigManager::validateConfig(const Config& config) const
{
  // Example validation rules
  if (config.param1 <= 0.0)
  {
    RCLCPP_ERROR(getLogger(), "[CpTemplateConfigManager] param1 must be positive");
    return false;
  }
  
  if (config.param2 < 0 || config.param2 > 100)
  {
    RCLCPP_ERROR(getLogger(), "[CpTemplateConfigManager] param2 must be between 0 and 100");
    return false;
  }
  
  if (config.param3.empty())
  {
    RCLCPP_ERROR(getLogger(), "[CpTemplateConfigManager] param3 cannot be empty");
    return false;
  }
  
  return true;
}

void CpTemplateConfigManager::resetToDefaults()
{
  std::lock_guard<std::mutex> lock(config_mutex_);
  current_config_ = default_config_;
  updateROSParameters();
  
  RCLCPP_INFO(getLogger(), "[CpTemplateConfigManager] Reset to default configuration");
}

void CpTemplateConfigManager::updateROSParameters()
{
  if (!node_)
    return;
  
  // Update ROS parameters based on current config
  node_->set_parameter(rclcpp::Parameter("template.param1", current_config_.param1));
  node_->set_parameter(rclcpp::Parameter("template.param2", current_config_.param2));
  node_->set_parameter(rclcpp::Parameter("template.param3", current_config_.param3));
  node_->set_parameter(rclcpp::Parameter("template.param4", current_config_.param4));
}

void CpTemplateConfigManager::loadROSParameters()
{
  if (!node_)
    return;
  
  // Declare and get parameters
  node_->declare_parameter("template.param1", current_config_.param1);
  node_->declare_parameter("template.param2", current_config_.param2);
  node_->declare_parameter("template.param3", current_config_.param3);
  node_->declare_parameter("template.param4", current_config_.param4);
  
  current_config_.param1 = node_->get_parameter("template.param1").as_double();
  current_config_.param2 = node_->get_parameter("template.param2").as_int();
  current_config_.param3 = node_->get_parameter("template.param3").as_string();
  current_config_.param4 = node_->get_parameter("template.param4").as_bool();
}

}  // namespace cl_template