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

#include <cl_rgb/components/cp_transforms.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cl_rgb
{

CpTransforms::CpTransforms() {}

CpTransforms::~CpTransforms() {}

void CpTransforms::onInitialize()
{
  auto node = this->getNode();
  buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  // no thread of its own: the transforms arrive on the state machine's executor
  listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, node, false);
}

std::optional<tf2::Transform> CpTransforms::lookup(
  const std::string & targetFrame, const std::string & sourceFrame,
  const rclcpp::Time & stamp) const
{
  try
  {
    const auto msg = buffer_->lookupTransform(
      targetFrame, sourceFrame, tf2::TimePoint(std::chrono::nanoseconds(stamp.nanoseconds())));
    tf2::Transform transform;
    tf2::fromMsg(msg.transform, transform);
    return transform;
  }
  catch (const tf2::TransformException &)
  {
    return std::nullopt;
  }
}

}  // namespace cl_rgb
