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
#include <cl_nav2z/client_behaviors/cb_wait_transform.hpp>

namespace cl_nav2z
{
CbWaitTransform::CbWaitTransform(
  std::string targetFrame, std::string referenceFrame, rclcpp::Duration timeout)
: targetFrame_(targetFrame), referenceFrame_(referenceFrame), timeout_(timeout)
{
}

CbWaitTransform::~CbWaitTransform() {}

void CbWaitTransform::onEntry()
{
  RCLCPP_INFO(
    getLogger(), "[CbWaitTransform] ref %s -> target %s", referenceFrame_.c_str(),
    targetFrame_.c_str());

  tfBuffer_ = std::make_shared<tf2_ros::Buffer>(getNode()->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  tf2::Stamped<tf2::Transform> transform;
  try
  {
    // the latest transform: this buffer is new, so it never holds data for the time of this call
    auto transformstamped =
      tfBuffer_->lookupTransform(targetFrame_, referenceFrame_, rclcpp::Time(0), timeout_);
    tf2::fromMsg(transformstamped, transform);

    result_ = transform;

    RCLCPP_INFO(
      getLogger(), "[CbWaitTransform] Success wait transform ref %s -> target %s",
      referenceFrame_.c_str(), targetFrame_.c_str());
    this->postSuccessEvent();
  }
  catch (tf2::TransformException & ex)
  {
    RCLCPP_ERROR_STREAM(
      getLogger(), "[CbWaitTransform] Failure waiting transform ( ref "
                     << referenceFrame_ << "/ target " << targetFrame_ << " - " << ex.what());
    this->postFailureEvent();
  }
}
}  // namespace cl_nav2z
