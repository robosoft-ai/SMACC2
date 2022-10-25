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

#include <nav2z_client/nav2z_client.hpp>
#include <smacc2/smacc_asynchronous_client_behavior.hpp>

namespace sm_husky_barrel_search_1
{


class CbCallback : public smacc2::SmaccAsyncClientBehavior
{
public:

  CbCallback(std::function<void()> callback):callback_(callback)
  {

  }

  virtual ~CbCallback()
  {

  }

  template <typename TOrthogonal, typename TSourceObject>
  void onOrthogonalAllocation()
  {
    smacc2::SmaccAsyncClientBehavior::onOrthogonalAllocation<TOrthogonal, TSourceObject>();
  }

  void onEntry() override
  {
    this->callback_();
    this->postSuccessEvent();
  }

protected:
  std::function<void()> callback_;
};
}  // namespace cl_nav2z
