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

#pragma once
#include <smacc2/smacc.hpp>

using namespace std::chrono_literals;

namespace sm_aws_warehouse_navigation
{
// STATE DECLARATION
struct StStartNavigation : smacc2::SmaccState<StStartNavigation, SmAwsWarehouseNavigation>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef mpl::list<

     Transition<EvCbSuccess<CbWaitNav2Nodes, OrNavigation>, StInitialNavigateForward, SUCCESS>
    // , Transition<EvActionAborted<ClNav2Z, OrNavigation>, StAcquireSensors, ABORT>
    //Transition<EvCbSuccess<OrNavigation, CbWaitTransform>, StInitialNavigateForward, SUCCESS> >
    >
    reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNavigation, CbRosLaunch>();
    configure_orthogonal<OrNavigation, CbWaitNav2Nodes>();
  }

  void runtimeConfigure()
  {
    // illegal wait workaround
    // rclcpp::sleep_for(6s);

    // ClNav2Z * navClient;
    // getOrthogonal<OrNavigation>()->requiresClient(navClient);

    // amcl_ = navClient->getComponent<Amcl>();
  }

  void onEntry()
  {
    //sendInitialPoseEstimation();
    //auto res = exec("ros2");
    //RCLCPP_INFO(getLogger(), "launch result: %s", res.c_str());
  }

  void sendInitialPoseEstimation()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped initialposemsg;
    // bool useSimTime = getNode()->get_parameter("use_sim_time").as_bool();
    //getNode()->set_parameter("use_sim_time",true);

    initialposemsg.header.stamp = getNode()->now();
    initialposemsg.header.frame_id = "map";

    initialposemsg.pose.pose.position.x = 3.415412425994873;
    initialposemsg.pose.pose.position.y = 2.0;
    initialposemsg.pose.pose.position.z = 0;

    initialposemsg.pose.pose.orientation.x = 0;
    initialposemsg.pose.pose.orientation.y = 0;
    initialposemsg.pose.pose.orientation.z = 1;
    initialposemsg.pose.pose.orientation.w = 0;

    //z: 0.9999985465626609
    // w: 0.00170495529732811

    initialposemsg.pose.covariance = {
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0,
      0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.06853891909122467};

    // amcl_->setInitialPose(initialposemsg);
  }

  void onExit() {}
};
}  // namespace sm_aws_warehouse_navigation
