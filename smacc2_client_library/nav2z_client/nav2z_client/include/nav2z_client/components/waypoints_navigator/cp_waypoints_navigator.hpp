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

#include <nav2z_client/components/waypoints_navigator/cp_waypoints_event_dispatcher.hpp>
#include <nav2z_client/nav2z_client.hpp>
#include <smacc2/smacc.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <nav2z_client/components/waypoints_navigator/cp_waypoints_navigator_base.hpp>

namespace cl_nav2z
{
class ClNav2Z;

struct EvWaypointFinal : sc::event<EvWaypointFinal>
{
};

struct NavigateNextWaypointOptions
{
  std::optional<std::string> controllerName_;
  std::optional<std::string> goalCheckerName_;
};

// This component contains a list of waypoints. These waypoints can
// be iterated in the different states using CbNextWaiPoint
// waypoint index is only incremented if the current waypoint is successfully reached
class CpWaypointNavigator : public CpWaypointNavigatorBase
{
public:
  ClNav2Z * client_;

  CpWaypointNavigator();

  void onInitialize() override;

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    waypointsEventDispatcher.initialize<TSourceObject, TOrthogonal>(client_);
  }

  std::optional<std::shared_future<
    std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose> > > >
  sendNextGoal(
    std::optional<NavigateNextWaypointOptions> options = std::nullopt,
    cl_nav2z::ClNav2Z::SmaccNavigateResultSignal::WeakPtr callback =
      cl_nav2z::ClNav2Z::SmaccNavigateResultSignal::WeakPtr());

  void stopWaitingResult();

  smacc2::SmaccSignal<void()> onNavigationRequestSucceded;
  smacc2::SmaccSignal<void()> onNavigationRequestAborted;
  smacc2::SmaccSignal<void()> onNavigationRequestCancelled;

private:
  void onNavigationResult(const ClNav2Z::WrappedResult & r);

  void onGoalReached(const ClNav2Z::WrappedResult & res);
  void onGoalCancelled(const ClNav2Z::WrappedResult & /*res*/);
  void onGoalAborted(const ClNav2Z::WrappedResult & /*res*/);

  boost::signals2::connection succeddedNav2ZClientConnection_;
  boost::signals2::connection abortedNav2ZClientConnection_;
  boost::signals2::connection cancelledNav2ZClientConnection_;
};
}  // namespace cl_nav2z
