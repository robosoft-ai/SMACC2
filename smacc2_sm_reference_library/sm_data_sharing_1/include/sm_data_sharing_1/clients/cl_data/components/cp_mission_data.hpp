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
#include <smacc2/component.hpp>

namespace sm_data_sharing_1
{
namespace cl_data
{

struct Position2D
{
  double x{0.0};
  double y{0.0};
};

// CpMissionData is a state-machine-scoped component that persists across all state transitions.
// Client behaviors in any orthogonal can access it via requiresComponent(), which performs a
// global search across all clients in all orthogonals.
class CpMissionData : public smacc2::ISmaccComponent
{
public:
  std::optional<Position2D> initialPosition;
  std::optional<Position2D> targetPosition;

  void onInitialize() override {}
};

}  // namespace cl_data
}  // namespace sm_data_sharing_1
