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

#include <cl_px4_mr/cl_px4_mr.hpp>
#include <cl_px4_mr/components/cp_tf_broadcaster.hpp>
#include <config/mission_constants.hpp>
#include <smacc2/smacc.hpp>

namespace sm_cl_px4_mr_test_5
{

// PX4 vehicle control, plus the TF tree (map -> base_link -> lidar_link) that
// places the lidar cloud in RViz
class OrPx4 : public smacc2::Orthogonal<OrPx4>
{
public:
  void onInitialize() override
  {
    auto client = this->createClient<cl_px4_mr::ClPx4Mr>();
    client->createComponent<cl_px4_mr::CpTfBroadcaster>(tfParams());
  }
};

}  // namespace sm_cl_px4_mr_test_5
