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
 *****************************************************************************************************************/

#include "cb_circular_pivot_motion.hpp"

namespace cl_move_group_interface
{
// spins the end effector joint (or any other arbitrary joint etting the tipLink parameter)
class CbEndEffectorRotate : public CbCircularPivotMotion
{
public:
  CbEndEffectorRotate(double deltaRadians, std::optional<std::string> tipLink = std::nullopt);

  virtual ~CbEndEffectorRotate();

  virtual void onEntry() override;

  std::optional<std::string> tipLink;
};

}  // namespace cl_move_group_interface
