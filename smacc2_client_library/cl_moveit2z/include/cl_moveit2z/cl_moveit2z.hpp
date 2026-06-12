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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 *****************************************************************************************************************/

#pragma once

#include <cl_moveit2z/components/cp_move_group_interface.hpp>
#include <smacc2/smacc.hpp>

namespace cl_moveit2z
{
class ClMoveit2z : public smacc2::ISmaccClient
{
public:
  ClMoveit2z(std::string groupName);

  ClMoveit2z(const moveit::planning_interface::MoveGroupInterface::Options & options);

  virtual ~ClMoveit2z();

  template <typename TOrthogonal, typename TClient>
  void onComponentInitialization()
  {
    this->createComponent<CpMoveGroupInterface, TOrthogonal, TClient>(options_);
  }

  inline const moveit::planning_interface::MoveGroupInterface::Options & getOptions() const
  {
    return options_;
  }

private:
  moveit::planning_interface::MoveGroupInterface::Options options_;
};
}  // namespace cl_moveit2z
