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

#include <smacc2/client_bases/smacc_service_client.hpp>
#include <std_srvs/srv/set_bool.hpp>
namespace sm_dance_bot_strikes_back
{
namespace cl_service3
{
class ClService3 : public smacc2::client_bases::SmaccServiceClient<std_srvs::srv::SetBool>
{
public:
  ClService3() {}
};
}  // namespace cl_service3
}  // namespace sm_dance_bot_strikes_back
