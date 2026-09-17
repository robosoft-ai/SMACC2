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

#include <smacc2/smacc.hpp>

// Railway dispatch event: StRailway posts EvRailwayDispatch<Target> from its
// runtimeConfigure and its transition table has one row per target. The
// template argument doubles as the "source" label in the transition log.

namespace sm_cl_px4_mr_test_4
{

using namespace smacc2::default_transition_tags;

template <typename TTarget>
struct EvRailwayDispatch : boost::statechart::event<EvRailwayDispatch<TTarget>>
{
};

// custom transition tags
struct NEXT : SUCCESS
{
};
struct DISPATCH : SUCCESS
{
};

}  // namespace sm_cl_px4_mr_test_4
