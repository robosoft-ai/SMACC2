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
 ******************************************************************************************************************/

#include "smacc2/common.hpp"
#include "smacc2/client_bases/smacc_action_client_base.hpp"

namespace smacc2
{
namespace utils
{
std::string cleanShortTypeName(const std::type_info & tinfo)
{
  auto typeinfo = TypeInfo::getFromStdTypeInfo(tinfo);
  auto nontemplatedfullclasname = typeinfo->getNonTemplatedTypeName();

  std::vector<std::string> strs;
  boost::split(strs, nontemplatedfullclasname, boost::is_any_of("::"));
  std::string classname = strs.back();

  return classname;
}
}  // namespace utils
}  // namespace smacc2
