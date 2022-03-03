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

#include <smacc2/client_bases/smacc_subscriber_client.hpp>
#include <std_msgs/msg/int32.hpp>

namespace sm_husky_barrel_search_1
{
namespace cl_opencv_perception
{
struct EvDetectedBarrelColor : sc::event<EvDetectedBarrelColor>
{
};

class ClOpenCVPerception : public smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::Int32>
{
public:
  ClOpenCVPerception(std::string topicname = "/detected_color")
  : smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::Int32>(topicname)
  {
  }

  virtual ~ClOpenCVPerception() {}

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    smacc2::client_bases::SmaccSubscriberClient<std_msgs::msg::Int32>::onOrthogonalAllocation<
      TOrthogonal, TSourceObject>();
  }
};
}  // namespace cl_opencv_perception
}  // namespace sm_husky_barrel_search_1
