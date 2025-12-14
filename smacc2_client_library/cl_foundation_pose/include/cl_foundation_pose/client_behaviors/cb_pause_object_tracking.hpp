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

#include <smacc2/smacc.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace cl_foundation_pose
{

class CbTrackObjectPause : public smacc2::SmaccClientBehavior , public smacc2::ISmaccUpdatable
{
private:
    cl_foundation_pose::CpObjectTrackerTf   *objectTracker_ = nullptr;

public:
  CbTrackObjectPose()
  {

  }

  virtual ~CbTrackObjectPose()
  {

  }

  virtual void onEntry() override
  {
    requiresComponent(objectTracker_);
    RCLCPP_INFO(getLogger(), "CbTrackObjectPose onEntry");
    objectTracker_->setEnabled(False);
    RCLCPP_INFO(getLogger(), "CbTrackObjectPose onEntry - enabled");
  }

  virtual void onExit() override
  {
    objectTracker_->setEnabled(false);
  }

  virtual void update() override
  {
    if (objectTracker_!=nullptr && objectTracker_->isEnabled())
    objectTracker_->updateAndGetGlobalPose(objectToTrackId_, globalFrame_);
  }
};
} // namespace cl_foundation_pose
