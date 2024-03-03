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



// #goal definition

// # Whether to perform all 4 stages: Headlands, Swath (Required), Route, Path
// bool generate_headland True
// bool generate_route True
// bool generate_path True

// # The field specification to use.
// # If using polygons, bounding polygon must be first, followed by inner cutouts
// # Both must specify if the data is cartesian or GPS coordinates
// # If using Row Coverage Server, must use gml field.
// bool use_gml_file False
// string gml_field
// opennav_coverage_msgs/Coordinates[] polygons
// string frame_id map

// # Modes of operation of each stage, if used
// opennav_coverage_msgs/HeadlandMode headland_mode
// opennav_coverage_msgs/SwathMode swath_mode
// opennav_coverage_msgs/RowSwathMode row_swath_mode
// opennav_coverage_msgs/RouteMode route_mode
// opennav_coverage_msgs/PathMode path_mode

// ---
// #result definition

// # Error codes
// # Note: The expected priority order of the errors should match the message order
// uint16 NONE=0
// uint16 INTERNAL_F2C_ERROR=801
// uint16 INVALID_MODE_SET=802
// uint16 INVALID_REQUEST=803
// uint16 INVALID_COORDS=803

// nav_msgs/Path nav_path
// opennav_coverage_msgs/PathComponents coverage_path
// builtin_interfaces/Duration planning_time
// uint16 error_code

// ---
// #feedback definition

#include <sm_dance_bot_coverage/clients/cl_navigation_coverage/cl_navigation_coverage.hpp>
#include <opennav_coverage_msgs/action/navigate_complete_coverage.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>
#include <opennav_coverage_msgs/msg/coordinates.hpp>

namespace sm_dance_bot_coverage
{
class CbNavigateCoverage : public smacc2::SmaccAsyncClientBehavior
{
private:
    sm_dance_bot_coverage::ClNavigateCoverage *actionClient_;

public:
    std::vector<geometry_msgs::msg::Polygon> perimeter_;
    std::string frame_id_;
    
public:
    CbNavigateCoverage()
    {
    }
    
    CbNavigateCoverage(std::vector<geometry_msgs::msg::Polygon> perimeter, std::string frame_id)
    {
        this->perimeter_ = perimeter;
        this->frame_id_ = frame_id;
    }

    void onEntry() override
    {
        this->requiresClient(actionClient_);

        auto goalMsg = std::make_shared<opennav_coverage_msgs::action::NavigateCompleteCoverage::Goal>();
        
        // goalMsg->goal.generate_headland = true;
        // goalMsg->generate_route = true;
        // goalMsg->generate_path = true;
        // goalMsg->use_gml_file = false;
        goalMsg->frame_id = this->frame_id_;

        // fill polygon
        goalMsg->polygons = this->perimeter_;

        this->actionClient_->sendGoal(*goalMsg);
    }
};
}
// action client behavior cb navigate coverage

