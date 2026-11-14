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

#include <cstdlib>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace sm_coretest_exceptions_1
{
// The site that throws in this run, from the environment: none (default), state_entry, state_exit,
// sync_entry, sync_exit, async_entry, async_exit, client_update or behavior_update.
inline const std::string & throwSite()
{
  static const std::string site = []
  {
    const char * value = std::getenv("SM_CORETEST_THROW_SITE");
    return std::string(value ? value : "none");
  }();
  return site;
}

// Logs and throws when the run's site is this one.
inline void throwIfSite(const rclcpp::Logger & logger, const char * site)
{
  if (throwSite() != site) return;
  RCLCPP_WARN(logger, "THROWING at %s", site);
  throw std::runtime_error(std::string("sm_coretest_exceptions_1: thrown at ") + site);
}
}  // namespace sm_coretest_exceptions_1
