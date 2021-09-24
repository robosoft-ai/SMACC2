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
#pragma once

#include <tf2/transform_datatypes.h>
#include <smacc2/common.hpp>
#include <smacc2/component.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <mutex>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>

// #include <realtime_tools/realtime_publisher.h>
// #include <dynamic_reconfigure/server.h>
//#include <move_base_z_client_plugin/OdomTrackerConfig.h>

namespace cl_move_base_z
{
namespace odom_tracker
{
enum class WorkingMode : uint8_t
{
  RECORD_PATH = 0,
  CLEAR_PATH = 1,
  IDLE = 2
};

/// This class track the required distance of the cord based on the external localization system
class OdomTracker : public smacc2::ISmaccComponent
{
public:
  // by default, the component start in record_path mode and publishing the
  // current path
  OdomTracker(std::string odomtopicName = "/odom", std::string odomFrame = "odom");

  // threadsafe
  /// odom callback: Updates the path - this must be called periodically for each odometry message.
  // The odom parameters is the main input of this tracker
  virtual void processOdometryMessage(const nav_msgs::msg::Odometry::SharedPtr odom);

  // ------ CONTROL COMMANDS ---------------------
  // threadsafe
  void setWorkingMode(WorkingMode workingMode);

  // threadsafe
  void setPublishMessages(bool value);

  // threadsafe
  void pushPath();

  // threadsafe
  void popPath(int pathCount = 1, bool keepPreviousPath = false);

  // threadsafe
  void clearPath();

  // threadsafe
  void setStartPoint(const geometry_msgs::msg::PoseStamped & pose);

  // threadsafe
  void setStartPoint(const geometry_msgs::msg::Pose & pose);

  // threadsafe
  nav_msgs::msg::Path getPath();

  void logStateString();

protected:
  void onInitialize() override;

  void updateConfiguration();
  //   dynamic_reconfigure::Server<move_base_z_client_plugin::OdomTrackerConfig> paramServer_;
  //   dynamic_reconfigure::Server<move_base_z_client_plugin::OdomTrackerConfig>::CallbackType f;
  //   void reconfigCB(move_base_z_client_plugin::OdomTrackerConfig &config, uint32_t level);

  virtual void rtPublishPaths(rclcpp::Time timestamp);

  // this is called when a new odom message is received in record path mode
  virtual bool updateRecordPath(const nav_msgs::msg::Odometry & odom);

  // this is called when a new odom message is received in clear path mode
  virtual bool updateClearPath(const nav_msgs::msg::Odometry & odom);

  void updateAggregatedStackPath();

  // -------------- OUTPUTS ---------------------
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robotBasePathPub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr robotBasePathStackedPub_;

  // --------------- INPUTS ------------------------
  // optional, this class can be used directly calling the odomProcessing method
  // without any subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;

  // -------------- PARAMETERS ----------------------
  /// How much distance there is between two points of the path
  double recordPointDistanceThreshold_;

  /// Meters
  double recordAngularDistanceThreshold_;

  /// rads
  double clearPointDistanceThreshold_;

  /// rads
  double clearAngularDistanceThreshold_;

  std::string odomFrame_;

  std::string odomTopicName_;

  // --------------- STATE ---------------
  // default true
  bool publishMessages;

  /// Processed path for the mouth of the reel
  nav_msgs::msg::Path baseTrajectory_;

  WorkingMode workingMode_;

  std::vector<nav_msgs::msg::Path> pathStack_;

  nav_msgs::msg::Path aggregatedStackPathMsg_;

  // subscribes to topic on init if true
  bool subscribeToOdometryTopic_;

  std::mutex m_mutex_;
};

/**
 ******************************************************************************************************************
 * p2pDistance()
 ******************************************************************************************************************
 */
inline double p2pDistance(
  const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  double dx = (p1.x - p2.x);
  double dy = (p1.y - p2.y);
  double dz = (p2.z - p2.z);
  double dist = sqrt(dx * dx + dy * dy + dz * dz);
  return dist;
}
}  // namespace odom_tracker
}  // namespace cl_move_base_z
