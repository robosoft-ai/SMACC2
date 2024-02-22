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
#include <smacc2/client_behaviors/cb_ros_stop_2.hpp>

namespace smacc2
{
namespace client_behaviors
{
std::vector<std::future<std::string>> CbRosStop2::detached_futures_;

CbRosStop2::CbRosStop2() {}

CbRosStop2::CbRosStop2(pid_t launchPid) {}

CbRosStop2::~CbRosStop2() {}

template <typename TOrthogonal, typename TSourceObject>
void onOrthogonalAllocation()
{
  smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
}

void CbRosStop2::onEntry()
{
  this->requiresClient(client_);
  client_->stop();
}

}  // namespace client_behaviors
}  // namespace smacc2
