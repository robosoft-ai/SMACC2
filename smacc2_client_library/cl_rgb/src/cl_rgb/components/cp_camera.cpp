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

#include <cl_rgb/components/cp_camera.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace cl_rgb
{

CpCamera::CpCamera(const CameraParams & params) : params_(params) {}

CpCamera::~CpCamera() {}

void CpCamera::onInitialize()
{
  this->requiresComponent(transforms_, smacc2::ComponentRequirement::HARD);
  auto node = this->getNode();
  // A machine that acts on single frames must not lose one: reliable, only the newest kept.
  const rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(2)).reliable();
  imageSubscriber_ = node->create_subscription<sensor_msgs::msg::Image>(
    params_.imageTopic, qos, std::bind(&CpCamera::onImage, this, std::placeholders::_1));
  infoSubscriber_ = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    params_.infoTopic, qos, std::bind(&CpCamera::onInfo, this, std::placeholders::_1));
  RCLCPP_INFO(
    getLogger(), "CpCamera '%s': images on %s, calibration on %s", params_.name.c_str(),
    params_.imageTopic.c_str(), params_.infoTopic.c_str());
}

void CpCamera::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  cv_bridge::CvImageConstPtr converted;
  try
  {
    converted = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (const cv_bridge::Exception & e)
  {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), *this->getNode()->get_clock(), 5000, "CpCamera '%s': cannot convert image: %s",
      params_.name.c_str(), e.what());
    return;
  }

  auto frame = std::make_shared<Frame>();
  frame->image = converted->image;
  frame->stamp = rclcpp::Time(msg->header.stamp, this->getNode()->get_clock()->get_clock_type());
  frame->frameId = msg->header.frame_id;
  FramePtr snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frame->sequence = ++sequence_;
    latest_ = frame;
    snapshot = latest_;
  }
  onFrame_(snapshot);
}

void CpCamera::onInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!calibrated_)
  {
    RCLCPP_INFO(
      getLogger(), "CpCamera '%s': calibrated, focal length %.1f px, %ux%u", params_.name.c_str(),
      msg->k[0], msg->width, msg->height);
  }
  fx_ = msg->k[0];
  fy_ = msg->k[4];
  cx_ = msg->k[2];
  cy_ = msg->k[5];
  calibrated_ = fx_ > 0.0 && fy_ > 0.0;
}

FramePtr CpCamera::latestFrame() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

std::optional<PinholeCamera> CpCamera::model(const Frame & frame) const
{
  PinholeCamera camera;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!calibrated_)
    {
      return std::nullopt;
    }
    camera.fx = fx_;
    camera.fy = fy_;
    camera.cx = cx_;
    camera.cy = cy_;
  }
  const auto pose = transforms_->lookup(params_.worldFrame, frame.frameId, frame.stamp);
  if (!pose)
  {
    return std::nullopt;
  }
  const tf2::Matrix3x3 & basis = pose->getBasis();
  for (int r = 0; r < 3; ++r)
  {
    for (int c = 0; c < 3; ++c)
    {
      camera.rotation(r, c) = basis[r][c];
    }
  }
  const tf2::Vector3 & origin = pose->getOrigin();
  camera.position = cv::Vec3d(origin.x(), origin.y(), origin.z());
  return camera;
}

}  // namespace cl_rgb
