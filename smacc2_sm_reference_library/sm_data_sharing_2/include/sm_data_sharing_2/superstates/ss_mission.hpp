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

#include <optional>
#include <smacc2/smacc.hpp>

namespace sm_data_sharing_2
{

struct Position2D
{
  double x{0.0};
  double y{0.0};
};

// Forward declarations of inner states required by the SmaccState template.
struct StAcquireData1;
struct StAcquireData2;
struct StProcessData;

// SsMission is a superstate that owns shared mission data.
//
// Data stored here persists for the entire lifetime of this superstate — across
// all transitions between StAcquireData1, StAcquireData2, and StProcessData.
//
// From a state class:   call context<SsMission>() directly (boost::statechart API).
// From a client behavior: call getCurrentState()->getParentState(), then
//                         dynamic_cast<SsMission*> — see cb_store_data1.hpp.
struct SsMission : smacc2::SmaccState<SsMission, SmDataSharing2, StAcquireData1>
{
  using SmaccState::SmaccState;

  std::optional<Position2D> initialPosition;
  std::optional<Position2D> targetPosition;
};

}  // namespace sm_data_sharing_2
