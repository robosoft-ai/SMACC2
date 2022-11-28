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

#include <smacc2/client_behaviors/cb_service_server_callback_base.hpp>
#include <smacc2/smacc_client_behavior.hpp>

namespace sm_atomic_services
{
    template <typename TSource, typename TOrthogonal>
    struct EvServiceRequestReceieved : sc::event<EvServiceRequestReceieved<TSource, TOrthogonal>> {};

    class CbServiceServer : public smacc2::CbServiceServerCallbackBase<std_srvs::srv::Empty>
    {
      public:
        void onServiceRequestReceived(const std::shared_ptr<typename std_srvs::srv::Empty::Request> req, std::shared_ptr<typename std_srvs::srv::Empty::Response> res) override
        {
            RCLCPP_INFO_STREAM(getLogger(), "CbServiceServer: service request received:" << (uint64_t)req.get());
            postEvRequestReceived();
            RCLCPP_INFO_STREAM(getLogger(), "CbServiceServer: response:" << (uint64_t)res.get());
        }

        template <typename TOrthogonal, typename TSourceObject>
        void onOrthogonalAllocation()
        {
          this->postEvRequestReceived = [=]()
          {
            this->template postEvent<EvServiceRequestReceieved<TSourceObject, TOrthogonal>>();
          };
        }
      private:
        std::function<void()> postEvRequestReceived;
  };
}
