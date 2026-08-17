// Copyright 2025 Robosoft Inc.
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

#include <cl_nav2z/cl_nav2z.hpp>
#include <cl_nav2z/components/nav2_action_interface/cp_nav2_action_interface.hpp>
#include <cl_nav2z/components/planner_switcher/cp_planner_switcher.hpp>
#include <smacc2/client_behavior_bases/cb_action_client_behavior_base.hpp>

namespace cl_nav2z
{
using namespace smacc2;

// Navigation behavior base, built on the generic action-client behavior
// template: goal sending (server-ready guard + rejection watchdog), result
// signal wiring (state machine thread), in-flight goal cancellation on early
// state exit and EvCbSuccess/EvCbFailure termination all come from
// CbActionClientBehaviorBase<NavigateToPose>. This class adds the
// nav-domain component (CpNav2ActionInterface, which posts the machine-scoped
// EvAction* navigation events) and keeps the historical nav-named result
// virtuals for existing overriders.
class CbNav2ZClientBehaviorBase : public smacc2::client_behavior_bases::CbActionClientBehaviorBase<
                                    nav2_msgs::action::NavigateToPose>
{
  using CbActionBase =
    smacc2::client_behavior_bases::CbActionClientBehaviorBase<nav2_msgs::action::NavigateToPose>;

public:
  virtual ~CbNav2ZClientBehaviorBase();

  template <typename TOrthogonal, typename TSourceObject>
  void onStateOrthogonalAllocation()
  {
    this->requiresComponent(nav2ActionInterface_, ComponentRequirement::HARD);

    // resolves actionClient_ and wires the result signals (see the template
    // header for the threading rationale); chain-break => bad_function_call
    CbActionBase::template onStateOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

protected:
  // Result handlers dispatched by the template. The base implementations
  // record the result code and post the behavior success/failure events;
  // derived classes may override to customize result handling (see
  // CbNavigateNextWaypointUntilReached).
  virtual void onNavigationActionSuccess(const components::CpNav2ActionInterface::WrappedResult &);
  virtual void onNavigationActionAbort(const components::CpNav2ActionInterface::WrappedResult &);

  // route the template's result handlers into the nav-named virtuals
  void onActionSuccess(const WrappedResult & result) override;
  void onActionAbort(const WrappedResult & result) override;

  components::CpNav2ActionInterface * nav2ActionInterface_ = nullptr;

  rclcpp_action::ResultCode navigationResult_ = rclcpp_action::ResultCode::UNKNOWN;
};

enum class SpinningPlanner
{
  Default,
  PureSpinning,
  Forward
};
}  // namespace cl_nav2z
