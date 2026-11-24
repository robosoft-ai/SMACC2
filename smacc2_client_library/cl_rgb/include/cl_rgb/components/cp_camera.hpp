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

#include <cl_rgb/components/cp_transforms.hpp>
#include <cl_rgb/geometry.hpp>
#include <cl_rgb/types.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <smacc2/smacc.hpp>

#include <mutex>
#include <optional>
#include <string>

namespace cl_rgb
{

struct CameraParams
{
  std::string name;                  // of the component, e.g. "cam_head"
  std::string imageTopic;            // sensor_msgs/Image, any 8 bit colour encoding
  std::string infoTopic;             // sensor_msgs/CameraInfo
  std::string worldFrame = "world";  // frame in which rays and points are given
};

// One camera: its latest image, and how its pixels relate to the world. The pose comes from
// the transform tree at the time of the image, so a camera on a moving arm is handled the
// same as a fixed one. Create one named component per camera.
class CpCamera : public smacc2::ISmaccComponent
{
public:
  explicit CpCamera(const CameraParams & params);
  ~CpCamera() override;

  void onInitialize() override;

  const CameraParams & params() const { return params_; }

  // the latest image; null until the first one arrives
  FramePtr latestFrame() const;

  // the camera as it was when the frame was taken; nothing while the calibration or the
  // transform at that time is missing
  std::optional<PinholeCamera> model(const Frame & frame) const;

  // called on the executor thread with every image
  smacc2::SmaccSignal<void(const FramePtr &)> onFrame_;

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void onInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  CameraParams params_;
  CpTransforms * transforms_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSubscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr infoSubscriber_;

  mutable std::mutex mutex_;
  FramePtr latest_;
  uint64_t sequence_ = 0;
  bool calibrated_ = false;
  double fx_ = 1.0, fy_ = 1.0, cx_ = 0.0, cy_ = 0.0;
};

}  // namespace cl_rgb
