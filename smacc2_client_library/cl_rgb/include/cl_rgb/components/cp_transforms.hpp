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

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>
#include <smacc2/smacc.hpp>

#include <memory>
#include <optional>
#include <string>

namespace cl_rgb
{

// The transform tree, shared by the cameras of the client.
class CpTransforms : public smacc2::ISmaccComponent
{
public:
  CpTransforms();
  ~CpTransforms() override;

  void onInitialize() override;

  // pose of the source frame in the target frame at the given time; nothing when not known
  std::optional<tf2::Transform> lookup(
    const std::string & targetFrame, const std::string & sourceFrame,
    const rclcpp::Time & stamp) const;

private:
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
};

}  // namespace cl_rgb
