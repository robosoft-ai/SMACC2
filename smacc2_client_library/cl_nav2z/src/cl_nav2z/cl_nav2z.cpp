// Copyright 2024 Robosoft Inc.
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

#include <cl_nav2z/cl_nav2z.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace cl_nav2z
{

// No implementation needed for the refactored client - all functionality
// is provided through components and the header-only interface

}  // namespace cl_nav2z

// Export the ClNav2Z class as type smacc2::ISmaccClient as an implementation of the ISmaccClient interface.
PLUGINLIB_EXPORT_CLASS(cl_nav2z::ClNav2Z, smacc2::ISmaccClient)
