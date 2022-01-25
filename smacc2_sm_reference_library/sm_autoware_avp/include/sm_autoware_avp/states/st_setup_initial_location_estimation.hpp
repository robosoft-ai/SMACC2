// Copyright 2021 MyName/MyCompany Inc.
// Copyright 2021 RobosoftAI Inc. (template)
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

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

// LOCAL CLIENTS
#include "sm_autoware_avp/clients/autoware_client/cl_autoware.hpp"

// ORTHOGONAL
#include "sm_autoware_avp/orthogonals/or_autoware_auto.hpp"

// CLIENT BEHAVIORS
#include "sm_autoware_avp/clients/autoware_client/client_behaviors/cb_navigate_global_position.hpp"
#include "sm_autoware_avp/clients/autoware_client/client_behaviors/cb_setup_initial_pose_estimation.hpp"
#include "smacc2/client_behaviors/cb_wait_topic_message.hpp"

#include "ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp"

namespace sm_autoware_avp
{
// SMACC2 clases
using smacc2::EvStateRequestFinish;
using smacc2::Transition;
using smacc2::default_transition_tags::SUCCESS;
using smacc2::default_transition_tags::ABORT;

using namespace smacc2::client_behaviors;
using namespace sm_autoware_avp::clients::autoware_client;
using namespace cl_ros_timer;

//--------------------------------------------------------------------------------
// STATE DECLARATION
struct StSetupInitialLocationEstimation
: smacc2::SmaccState<StSetupInitialLocationEstimation, SmAutowareAvp>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef boost::mpl::list<
    //Transition<smacc2::EvCbSuccess<CbWaitLocalizationMessage, OrAutowareAuto>, StNavigateWaypoint1, SUCCESS>
    Transition<sm_autoware_avp::clients::EvAutoLocalized<sm_autoware_avp::clients::ClAutoware, OrAutowareAuto>, StNavigateWaypoint1, SUCCESS> ,
    Transition<EvTimer<CbTimerCountdownOnce, OrTimer>, StSetupInitialLocationEstimation, ABORT>
    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped initialLocationEstimation;

    initialLocationEstimation.pose.pose.position.x = -61.46;
    initialLocationEstimation.pose.pose.position.y = -41.25;
    initialLocationEstimation.pose.pose.position.z = 0;

    initialLocationEstimation.pose.pose.orientation.x = 0;
    initialLocationEstimation.pose.pose.orientation.y = 0;
    initialLocationEstimation.pose.pose.orientation.z = -0.9851278757214282;
    initialLocationEstimation.pose.pose.orientation.w = 0.171822782181485;

    initialLocationEstimation.pose.covariance = {
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0,
      0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.06853891945200942};

    configure_orthogonal<OrAutowareAuto, CbSetupInitialPoseEstimation>(initialLocationEstimation);
    configure_orthogonal<OrTimer, CbTimerCountdownOnce>(5);

  }

  void runtimeConfigure() { RCLCPP_INFO(getLogger(), "Entering StNavigateWaypoint1"); }

  void onEntry() { RCLCPP_INFO(getLogger(), "On Entry!"); }

  void onExit() { RCLCPP_INFO(getLogger(), "On Exit!"); }
};
}  // namespace sm_autoware_avp
