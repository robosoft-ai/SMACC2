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

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <cmath>

namespace cl_px4_mr
{

// Roll/pitch-only rotation from a PX4 attitude quaternion (order w, x, y, z,
// FRD body -> NED earth): maps a body-FLU vector (x forward, y left, z up -
// the frame of a level-mounted sensor) into a LEVEL frame with the same
// heading: x = horizontal forward, y = horizontal left, z = up. Row-major 3x3.
inline void levelRotationFromPx4Quaternion(const std::array<float, 4> & q, std::array<float, 9> & r)
{
  // px4_ros_com frame_transforms constants (tf2 order x, y, z, w)
  const tf2::Quaternion kNedEnuQ(0.70710678118654752, 0.70710678118654752, 0.0, 0.0);
  const tf2::Quaternion kFrdFluQ(1.0, 0.0, 0.0, 0.0);
  const tf2::Quaternion qNedFrd(q[1], q[2], q[3], q[0]);
  tf2::Quaternion qEnuFlu = kNedEnuQ * qNedFrd * kFrdFluQ;
  qEnuFlu.normalize();
  const tf2::Matrix3x3 m(qEnuFlu);
  const double yaw = std::atan2(m[1][0], m[0][0]);
  tf2::Matrix3x3 unyaw;
  unyaw.setRPY(0.0, 0.0, -yaw);
  const tf2::Matrix3x3 level = unyaw * m;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j)
    {
      r[i * 3 + j] = static_cast<float>(level[i][j]);
    }
  }
}

}  // namespace cl_px4_mr
