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

#include <sm_dance_bot_strikes_back/clients/cl_service3/cl_service3.hpp>
#include <smacc2/smacc_client_behavior.hpp>

namespace sm_dance_bot_strikes_back
{
namespace cl_service3
{
enum class Service3Command
{
  SERVICE3_ON,
  SERVICE3_OFF
};

class CbService3 : public smacc2::SmaccClientBehavior
{
private:
  ClService3 * serviceClient_;
  Service3Command value_;

public:
  CbService3(Service3Command value) { value_ = value; }

  void onEntry() override
  {
    this->requiresClient(serviceClient_);

    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    if (value_ == Service3Command::SERVICE3_ON)
      req->data = true;
    else
      req->data = false;

    serviceClient_->call(req);
  }
};
}  // namespace cl_service3
}  // namespace sm_dance_bot_strikes_back
