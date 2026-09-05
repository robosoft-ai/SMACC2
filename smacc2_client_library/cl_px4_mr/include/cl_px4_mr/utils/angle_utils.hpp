// Copyright 2026 RobosoftAI Inc.
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

#include <cmath>

namespace cl_px4_mr
{

// Wrap an angle to [-pi, pi]. PX4 heading and yaw setpoints are absolute wrapped
// angles; any arithmetic on them (base heading + sine offset) must re-wrap before
// being commanded or compared.
inline float wrapPi(float angle)
{
  angle = std::fmod(angle + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI));
  if (angle < 0.0f)
  {
    angle += 2.0f * static_cast<float>(M_PI);
  }
  return angle - static_cast<float>(M_PI);
}

}  // namespace cl_px4_mr
