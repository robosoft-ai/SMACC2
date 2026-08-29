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

#include <smacc2/smacc_client.hpp>
#include <smacc2/client_bases/smacc_action_client_base.hpp>
#include <opennav_coverage_msgs/action/navigate_complete_coverage.hpp>

namespace sm_dance_bot_coverage
{


typedef smacc2::client_bases::SmaccActionClientBase<opennav_coverage_msgs::action::NavigateCompleteCoverage> Base;
typedef Base::WrappedResult WrappedResult;


class ClNavigateCoverage : public smacc2::client_bases::SmaccActionClientBase<opennav_coverage_msgs::action::NavigateCompleteCoverage>
{
public:
  using smacc2::client_bases::SmaccActionClientBase<opennav_coverage_msgs::action::NavigateCompleteCoverage>::GoalHandle;
  using smacc2::client_bases::SmaccActionClientBase<opennav_coverage_msgs::action::NavigateCompleteCoverage>::ResultCallback;
  using smacc2::client_bases::SmaccActionClientBase<opennav_coverage_msgs::action::NavigateCompleteCoverage>::FeedbackCallback;

  ClNavigateCoverage(std::string coverage) : Base(coverage)
  {
  } 
};
} // namespace sm_dance_bot_coverage