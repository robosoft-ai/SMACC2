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

#pragma once

#include <smacc2/smacc_client.hpp>
#include "components/cp_mission_data.hpp"

namespace sm_data_sharing_1
{
namespace cl_data
{

// ClData is a pure orchestrator client whose only job is to own CpMissionData.
// It holds no business logic — all data access is done through the component.
class ClData : public smacc2::ISmaccClient
{
public:
  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    this->createComponent<CpMissionData, TOrthogonal, ClData>();
  }
};

}  // namespace cl_data
}  // namespace sm_data_sharing_1
